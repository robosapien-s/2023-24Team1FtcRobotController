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


    final double locationStepSize = 1.65;
    final double location1 = 4.5;
    final double location2 = 10.5;
    final double location3 = 16.5;


    public Integer currentDropLevel = 0;

    private  Integer currentDropLocation = 0;


    private final FtcDashboard dashboard = FtcDashboard.getInstance();
    TelemetryPacket packet  = new TelemetryPacket();

    public LedController ledController;

    public AutoDropOffController(IMUWrapper inDriveController, NeoArmWrapper inArmWrapper) {
        driveController = inDriveController;
        armWrapper = inArmWrapper;
    }


    public void initAprilTag(HardwareMap hardwareMap) {

        ledController =  new LedController(hardwareMap);

        // Create the AprilTag processor the easy way.
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();

        // Create the vision portal the easy way.
        if (USE_WEBCAM) {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTag);
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
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
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

                    DropData dropData = getTargetYData(); //Distance, Ext, Act

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

                    telemetry.addData("Target",xTargetPosition);
                    telemetry.addData("Current",xCurrentLocation);


                    armWrapper.SetLinearActuator(GetArmActuatorLocation());
                    armWrapper.SetLinearExtensionPos(GetArmExtensionLocation());
                }

                distanceController.setCurrentPosition(yEncoder.getCurrentPosition());
                locationController.setCurrentPosition(xEncoder.getCurrentPosition());

                packet.put("current location", distanceController.getCurrentPosition());

            } else {


                if(closetsDetection != null) {
                    DropData dropData = getTargetYData(); //distance, act, ext
                    distanceController.setTargetPosition(dropData.boardDistance);
                    distanceController.setCurrentPosition(closetsDetection.ftcPose.y);

                    locationController.setTargetPosition(getTargetLocation());
                    locationController.setCurrentPosition(getCurrentLocation(closetsDetection));

                    armWrapper.SetLinearActuator(GetArmActuatorLocation());
                    armWrapper.SetLinearExtensionPos(GetArmExtensionLocation());
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
                //driveController.setAutoMode(power, locationPower);
                driveController.setAutoMode(-power, joystickWrapper.gamepad1GetLeftStickY());
            } else if(NewDrive.tune_y) {
                driveController.setAutoMode(-power, 0);
            } else if(NewDrive.tune_x) {
                //driveController.setAutoMode(0, locationPower);
                driveController.setAutoMode(0, joystickWrapper.gamepad1GetLeftStickY());
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

    public DropData getTargetYData() {


        DropData dropData = new DropData(NewDrive.height0_Act, NewDrive.height0_Ext, NewDrive.height0_Distance);

        /*switch (currentDropLevel) {
            case 0:
                dropData = new DropData(NewDrive.height0_Act, NewDrive.height0_Ext, NewDrive.height0_Distance);
                break;
            case 1:
                dropData = new DropData(NewDrive.height1_Act, NewDrive.height1_Ext, NewDrive.height1_Distance);
                break;
            case 2:
                dropData = new DropData(NewDrive.height2_Act, NewDrive.height2_Ext, NewDrive.height2_Distance);
                break;
            case 3:
                dropData = new DropData(NewDrive.height3_Act, NewDrive.height3_Ext, NewDrive.height3_Distance);
                break;
            case 4:
                dropData = new DropData(NewDrive.height4_Act, NewDrive.height4_Ext, NewDrive.height4_Distance);
                break;
            default:
                dropData = new DropData(NewDrive.height0_Act, NewDrive.height0_Ext, NewDrive.height0_Distance);
                break;
        }*/


        dropData = new DropData(NewDrive.height0_Act, NewDrive.height0_Ext, NewDrive.height0_Distance);
        return dropData;

    }

    public double getTargetLocation() {

        //Init in the middle somewhere
        double location = 5*locationStepSize + locationStepSize;


        location = NewDrive.dropLocation0;
        /*
        switch (currentDropLocation){
            case 0:
                location = NewDrive.dropLocation0;
                break;
            case 1:
                location = NewDrive.dropLocation1;
                break;
            case 2:
                location = NewDrive.dropLocation2;
                break;
            case 3:
                location = NewDrive.dropLocation3;
                break;
            case 4:
                location = NewDrive.dropLocation4;
                break;
            case 5:
                location = NewDrive.dropLocation5;
                break;
            case 6:
                location = NewDrive.dropLocation6;
                break;
            case 7:
                location = NewDrive.dropLocation7;
                break;
            case 8:
                location = NewDrive.dropLocation8;
                break;
            case 9:
                location = NewDrive.dropLocation9;
                break;
            case 10:
                location = NewDrive.dropLocation10;
                break;
            case 11:
                location = NewDrive.dropLocation11;
                break;
            case 12:
                location = NewDrive.dropLocation12;
                break;
            default:
                location = NewDrive.dropLocation6;
                break;
        }*/
        return location;


        //return currentDropLocation*locationStepSize + locationStepSize;
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

        //ledController.setCurrentIndex(currentDropLocation);
    }

    public void setPreviousDropLocation() {
        currentDropLocation--;
        if(currentDropLocation < 0) {
            currentDropLocation = 12;
        }

        //ledController.setCurrentIndex(currentDropLocation);
    }


    public void setNextDropLevel() {
        currentDropLevel++;
        if(currentDropLevel > 4) {
            currentDropLevel = 0;
        }
        ledController.setCurrentIndex(currentDropLevel);
    }

    public void setPreviousDropLeve() {
        currentDropLevel--;
        if(currentDropLevel < 0) {
            currentDropLevel = 4;
        }
        ledController.setCurrentIndex(currentDropLevel);
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


    public int GetArmActuatorLocation(){
        switch (currentDropLevel){
            case 0:
                return 1170;
            case 1:
                return 1495;
            case 2:
                return 2068;
            case 3:
                return 2223;
        }
        return 0;
    }
    public int GetArmExtensionLocation(){
        switch (currentDropLevel){
            case 0:
                return 1580;
            case 1:
                return 1700;
            case 2:
                return 1994;
            case 3:
                return 2299;
        }
        return 0;
    }

}
