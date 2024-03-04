package org.firstinspires.ftc.teamcode.wrappers;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.encoderTicksToInches;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.OpModes.IMUWrapper;
import org.firstinspires.ftc.teamcode.OpModes.NewDrive;
import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.util.Encoder;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

public class AutoDropOffController {

    public enum EDropLocation {
        P_0_1,
        P_1,
        P_1_2,
        P_2,
        P_2_3,
        P_3,
        P_3_4,
        P_4,
        P_4_5,
        P_5,
        P_5_6,
        P_6,
        P_6_7
    }

    public class DropData {

        public DropData(int inArmActuator, int inArmExtension, double inBoardDistance) {
            armActuator = inArmActuator;
            armExtension = inArmExtension;
            boardDistance = inBoardDistance;
        }

        int armActuator = 0;
        int armExtension = 0;
        double boardDistance = 0;
    }


    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    /**
     * The variable to store our instance of the AprilTag processor.
     */
    private AprilTagProcessor aprilTag;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;

    private boolean isAutoMode = false;


    IMUWrapper driveController;
    NeoArmWrapper armWrapper;


    SimplePIDController distanceController = new SimplePIDController(0.00035, 0.0, .00006);
    SimplePIDController locationController = new SimplePIDController(0.00046, 0.00000008, 0.00006);


    final double locationStepSize = 1.5;
    final double location1 = 4.5;
    final double location2 = 10.5;
    final double location3 = 16.5;


    private  Integer currentDropLevel = 0;
    private ArrayList<DropData> dropDistances = new ArrayList<DropData>();


    private  Integer currentDropLocation = 0;
    private HashMap<Integer, EDropLocation> dropLocations = new HashMap<Integer, EDropLocation>();


    private final FtcDashboard dashboard = FtcDashboard.getInstance();
    TelemetryPacket packet  = new TelemetryPacket();

    LedController ledController;

    public AutoDropOffController(IMUWrapper inDriveController, NeoArmWrapper inArmWrapper) {
        driveController = inDriveController;
        armWrapper = inArmWrapper;

        dropDistances.add( new DropData(612, 1945, 12.7));
        dropDistances.add( new DropData(1141, 2013, 11.3));
        dropDistances.add( new DropData(2660, 2005, 9));
        dropDistances.add( new DropData(1000, 1000, 24));
        dropDistances.add( new DropData(1200, 1200, 28));

        dropLocations.put(0, EDropLocation.P_0_1);
        dropLocations.put(1, EDropLocation.P_1);
        dropLocations.put(2, EDropLocation.P_1_2);
        dropLocations.put(3, EDropLocation.P_2);
        dropLocations.put(4, EDropLocation.P_2_3);
        dropLocations.put(5, EDropLocation.P_3);
        dropLocations.put(6, EDropLocation.P_3_4);
        dropLocations.put(7, EDropLocation.P_4);
        dropLocations.put(8, EDropLocation.P_4_5);
        dropLocations.put(9, EDropLocation.P_5);
        dropLocations.put(10, EDropLocation.P_5_6);
        dropLocations.put(11, EDropLocation.P_6);
        dropLocations.put(12, EDropLocation.P_6_7);

    }


    public void initAprilTag(HardwareMap hardwareMap) {

        ledController =  new LedController(hardwareMap);

        // Create the AprilTag processor the easy way.
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();

        // Create the vision portal the easy way.
        if (USE_WEBCAM) {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    hardwareMap.get(WebcamName.class, "Webcam 2"), aprilTag);
        } else {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    BuiltinCameraDirection.BACK, aprilTag);
        }
    }

    public void telemetryAprilTag(Telemetry telemetry, JoystickWrapper joystickWrapper, Encoder yEncoder, Encoder xEncoder) {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());
        telemetry.addLine(String.format("Current Location P: %d",currentDropLocation));
        telemetry.addLine(String.format("Current Level P: %d", currentDropLevel));

        double closest = 10000;

        boolean found1 = false;
        boolean found2 = false;
        boolean found3 = false;

        AprilTagDetection closetsDetection = null;


        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {

                if(detection.id == 1 || detection.id == 4) {
                    found1 = true;
                } else if (detection.id == 2 || detection.id == 5) {
                    found2 = true;
                } else if(detection.id == 3 || detection.id == 6) {
                    found3 = true;
                }


                if(closetsDetection ==  null) {
                    closetsDetection = detection;
                } else {
                    if(detection.ftcPose.y < closetsDetection.ftcPose.y) {
                        closetsDetection = detection;
                    }
                }


                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                //telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                //telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                //telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));


            } else {
                /*
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
                 */
            }
        }   // end for() loop

        /*
        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");
        */
        boolean usingEncoders = true;
        double yPowerFactor = 1;
        double xPowerFactor = -1;

        if(joystickWrapper.gamepad1GetLeftStick() && (closetsDetection != null || (distanceController.isEnabled()&&usingEncoders)) ) {


            //Get the initial error, then use the encoders to do the movements, i think the camera doesn't update as quickly as needed


            if (usingEncoders){

                yPowerFactor = -1;
                if (!distanceController.isEnabled()) {

                    DropData dropData = dropDistances.get(currentDropLevel);

                    double yDistanceNeededToTravel = closetsDetection.ftcPose.y - dropData.boardDistance;
                    double boardDistanceTicks = StandardTrackingWheelLocalizer.inchesToEncoderTicks(yDistanceNeededToTravel);
                    double yTargetPosition = yEncoder.getCurrentPosition() + boardDistanceTicks; //need to figure out direction
                    distanceController.setTargetPosition(yTargetPosition);


                    double xTargetLocation =  getTargetLocation();
                    double xCurrentLocation = getCurrentLocation(closetsDetection);


                    double xDistanceNeededToTravel = xTargetLocation - xCurrentLocation;


                    double xDistanceTicks = StandardTrackingWheelLocalizer.inchesToEncoderTicks(xDistanceNeededToTravel);
                    double xTargetPosition = xEncoder.getCurrentPosition() + xDistanceTicks;
                    locationController.setTargetPosition(xTargetPosition);

                    packet.put("target location", getTargetLocation());

                    armWrapper.SetLinearActuator(dropData.armActuator);
                    armWrapper.SetLinearExtensionPos(dropData.armExtension);
                }

                distanceController.setCurrentPosition(yEncoder.getCurrentPosition());
                locationController.setCurrentPosition(xEncoder.getCurrentPosition());

                packet.put("current location", distanceController.getCurrentPosition());

            } else {


                if(closetsDetection != null) {
                    DropData dropData = dropDistances.get(currentDropLevel);
                    distanceController.setTargetPosition(dropData.boardDistance);
                    distanceController.setCurrentPosition(closetsDetection.ftcPose.y);

                    locationController.setTargetPosition(getTargetLocation());
                    locationController.setCurrentPosition(getCurrentLocation(closetsDetection));

                    armWrapper.SetLinearActuator(dropData.armActuator);
                    armWrapper.SetLinearExtensionPos(dropData.armExtension);
                }
            }

            double yaw = driveController.getYaw();
            if(driveController.getYaw()<0) {
                // xDistanceNeededToTravel =  xCurrentLocation - xTargetLocation;
                xPowerFactor = 1;
                yPowerFactor = 1;
            }


            //Needs to be set based on alliance side
            //double power = distanceController.getPower(closetsDetection.ftcPose.y);
            double power = distanceController.getPowerWithStates(NewDrive.Kp_y, NewDrive.Ki_y, NewDrive.Kd_y) * yPowerFactor;

            //telemetry.addLine(String.format("Dist - Cur: %6.3f   Des: %6.3f   P: %6.3f", distanceController.getCurrentPosition(), distanceController.getTargetPosition(), power));



            double locationPower = locationController.getPowerWithStates(NewDrive.Kp_x, NewDrive.Ki_x, NewDrive.Kd_x) * xPowerFactor;
            //telemetry.addLine(String.format("Loc - Cur: %6.3f   Des: %6.3f   P: %6.3f", locationController.getCurrentPosition(), getTargetLocation(), locationPower));


            packet.put("power", power);
            packet.put("position", distanceController.getCurrentPosition());
            packet.put("target", distanceController.getTargetPosition());
            packet.put("error", distanceController.getLastError());


            packet.put("l_power", locationPower);
            packet.put("l_position",  locationController.getCurrentPosition());
            packet.put("l_error", locationController.getLastError());


            if(NewDrive.tune_y && NewDrive.tune_x) {
                driveController.setAutoMode(power, locationPower);
            } else if(NewDrive.tune_y) {
                driveController.setAutoMode(power, 0);
            } else if(NewDrive.tune_x) {
                driveController.setAutoMode(0, locationPower);
            } else {

            }





            dashboard.sendTelemetryPacket(packet);


        } else {



            packet.put("Y pos",yEncoder.getCurrentPosition());
            //telemetry.addLine(String.format("Current Y Pos %d",yEncoder.getCurrentPosition()));

            dashboard.sendTelemetryPacket(packet);
            driveController.disableAutoMode();
            distanceController.clearTarget();
        }
    }



    /*
    public double returnPower(double reference, double state) {
        double error = reference - state;
        double seconds = timer.seconds();
        intergralSum += error * seconds;

        double errorDiff = error - lastError;
        double derivative = errorDiff / timer.seconds();

        derivative = (error - lastError) / seconds;

        lastError = error;
        timer.reset();
        double output = (error*Kp) + (derivative*Kd) + (intergralSum*Ki);

        if(output>1) {
            output = 1;
        }
        return  output;
    }

    public double returnDropLocationPower(double reference, double state) {
        double error = reference - state;
        double seconds = timer.seconds();
        intergralSum += error * seconds;

        double errorDiff = error - lastError;
        double derivative = errorDiff / timer.seconds();

        derivative = (error - lastError) / seconds;

        lastError = error;
        timer.reset();
        double output = (error*Kp) + (derivative*Kd) + (intergralSum*Ki);

        if(output>1) {
            output = 1;
        }
        return  output;
    }
    */

    public double getTargetLocation() {
        return currentDropLocation*locationStepSize;
    }
    public double getCurrentLocation(AprilTagDetection closetsDetection) {

        double currentLocation = 0;

        if(closetsDetection.id == 1 || closetsDetection.id == 4) {
            currentLocation = location1 - closetsDetection.ftcPose.x;
        } else if(closetsDetection.id == 2 || closetsDetection.id == 5) {
            currentLocation = location2 - closetsDetection.ftcPose.x;
        } else if(closetsDetection.id == 3 || closetsDetection.id == 6) {
            currentLocation = location3 - closetsDetection.ftcPose.x;
        }

        return currentLocation;
    }


    public void setNextDropLocation() {
        currentDropLocation++;
        if(currentDropLocation > 12) {
            currentDropLocation = 0;
        }

        ledController.setCurrentIndex(currentDropLocation);
    }

    public void setPreviousDropLocation() {
        currentDropLocation--;
        if(currentDropLocation < 0) {
            currentDropLocation = 12;
        }

        ledController.setCurrentIndex(currentDropLocation);
    }



    public void setNextDropLevel() {
        currentDropLevel++;
        if(currentDropLevel > 4) {
            currentDropLevel = 0;
        }
    }

    public void setPreviousDropLeve() {
        currentDropLevel--;
        if(currentDropLevel < 0) {
            currentDropLevel = 4;
        }
    }




//    public void setDropLocation() {
//        currentDropLocation--;
//        if(currentDropLocation < 0) {
//            currentDropLocation = 12;
//        }
//    }
    // en

    public void setDropLevel1() {
        isAutoMode = true;
        distanceController.setTargetPosition(20);
    }

    public void setDropLevel2() {
        isAutoMode = true;
        distanceController.setTargetPosition(10);
    }

    public void close() {
        visionPortal.close();
    }


}
