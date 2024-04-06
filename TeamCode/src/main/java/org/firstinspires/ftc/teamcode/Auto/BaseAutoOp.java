package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.OpModes.InitActuatorPos;
import org.firstinspires.ftc.teamcode.OpModes.NewDrive;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.ITrajectorySequenceUpdateCallback;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.wrappers.BlueClosePropWrapper;
import org.firstinspires.ftc.teamcode.wrappers.BlueFarPropWrapper;
import org.firstinspires.ftc.teamcode.wrappers.NeoArmWrapper;
import org.firstinspires.ftc.teamcode.wrappers.RedClosePropWrapper;
import org.firstinspires.ftc.teamcode.wrappers.RedFarPropWrapper;

public abstract class BaseAutoOp extends LinearOpMode implements ITrajectorySequenceUpdateCallback {


    IMU imu;

    Pose2d startPose = new Pose2d(15,-62, Math.toRadians(-90));


    RedClosePropWrapper redClosePropWrapper;

    RedFarPropWrapper redFarPropWrapper;

    BlueClosePropWrapper blueClosePropWrapper;

    BlueFarPropWrapper blueFarPropWrapper;

    NeoArmWrapper neoArmWrapper;

    SampleMecanumDrive drive = null;

    static int barcodeInt;


    public void initialize(Pose2d inStartPose, int inBarcodeInt) {


        startPose = inStartPose;

        redClosePropWrapper = new RedClosePropWrapper(hardwareMap,telemetry);

        redFarPropWrapper = new RedFarPropWrapper(hardwareMap, telemetry);

        blueClosePropWrapper = new BlueClosePropWrapper(hardwareMap, telemetry);

        blueFarPropWrapper = new BlueFarPropWrapper(hardwareMap, telemetry);


        drive = new SampleMecanumDrive(hardwareMap, this);

        neoArmWrapper = new NeoArmWrapper(telemetry, hardwareMap, gamepad1, gamepad2, true);
        neoArmWrapper.ResetMotorPositions();

        neoArmWrapper.MoveActuatorMotor(InitActuatorPos.actuatorPos);

        drive.setPoseEstimate(startPose);

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        // Now initialize the IMU with this mounting orientation
        // This sample expects the IMU to be in a REV Hub and named "imu".
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();
        NewDrive.reset_imu = false;

    }

    public void run(){
    }

    @Override
    public void  update() {

    }

    protected TrajectorySequenceBuilder setInitialPose(Pose2d pose2d) {
        return drive.trajectorySequenceBuilder(pose2d);
    }

    protected TrajectorySequenceBuilder getArmReady( TrajectorySequenceBuilder sequenceBuilder) {
        return sequenceBuilder.setReversed(true)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    neoArmWrapper.OpenPos();
                    neoArmWrapper.WristDown();
                });
    }

    protected TrajectorySequenceBuilder dropPurplePixel( TrajectorySequenceBuilder sequenceBuilder, Vector2d endPosition, double heading) {
         return sequenceBuilder

                 .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                     neoArmWrapper.MoveActuatorMotor(-10);
                 })

                 .UNSTABLE_addTemporalMarkerOffset(1.5, () -> {
                     neoArmWrapper.ResetMotorPositions();
                 })

                 .splineTo(
                         endPosition,
                         heading,
                         SampleMecanumDrive.getVelocityConstraint(20,20,DriveConstants.TRACK_WIDTH),
                         SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)).setReversed(false)
                 .UNSTABLE_addTemporalMarkerOffset(.1, () -> {
            neoArmWrapper.ClosePos();
        })
                .UNSTABLE_addTemporalMarkerOffset(.3, () -> {
                    neoArmWrapper.WristUp();
                })
                .waitSeconds(1);
    }


    protected TrajectorySequenceBuilder getArmReadyFar( TrajectorySequenceBuilder sequenceBuilder) {
        return sequenceBuilder
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    neoArmWrapper.OpenPos();
                    neoArmWrapper.WristDown();
                });
    }

    protected TrajectorySequenceBuilder dropPurplePixelFar( TrajectorySequenceBuilder sequenceBuilder, Vector2d endPosition, double heading) {
        return sequenceBuilder
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    neoArmWrapper.MoveActuatorMotor(-10);
                })

                .UNSTABLE_addTemporalMarkerOffset(1.5, () -> {
                    neoArmWrapper.ResetMotorPositions();
                })
                .lineToLinearHeading( new Pose2d(endPosition.getX(), endPosition.getY(), heading),
                        SampleMecanumDrive.getVelocityConstraint(30,30,DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(.1, () -> {
                    neoArmWrapper.WristDown();
                })
                .UNSTABLE_addTemporalMarkerOffset(.1, () -> {
                    neoArmWrapper.OpenPos();
                });

    }

    protected TrajectorySequenceBuilder lineUpForSinglePixel( TrajectorySequenceBuilder sequenceBuilder, Vector2d endPosition, double heading) {
        return sequenceBuilder
                .waitSeconds(2)
                .lineToLinearHeading( new Pose2d(endPosition.getX(), endPosition.getY(), Math.toRadians(heading)))
                .UNSTABLE_addTemporalMarkerOffset(-2, () -> {
                    neoArmWrapper.ClosePos();
                })
                .UNSTABLE_addTemporalMarkerOffset(-1.5, () -> {
                    neoArmWrapper.WristUp();
                    neoArmWrapper.armWristServo.setPosition(NeoArmWrapper.arm_wrist_intake_pos);
                });
    }


    protected TrajectorySequenceBuilder lineUpForSinglePixelMiddle( TrajectorySequenceBuilder sequenceBuilder, Vector2d endPosition, double heading) {
        return sequenceBuilder
                .waitSeconds(2)
                .forward(2)
                .waitSeconds(.1)
                .turn(Math.toRadians(180))
                .waitSeconds(.1)
                /*.lineToLinearHeading( new Pose2d(endPosition.getX(), endPosition.getY(), Math.toRadians(heading)),
                        SampleMecanumDrive.getVelocityConstraint(30,30,DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))*/
                .UNSTABLE_addTemporalMarkerOffset(-2.5, () -> {
                    neoArmWrapper.ClosePos();
                })
                .UNSTABLE_addTemporalMarkerOffset(-2, () -> {
                    neoArmWrapper.WristUp();
                    neoArmWrapper.armWristServo.setPosition(NeoArmWrapper.arm_wrist_intake_pos);
                });
    }

    protected TrajectorySequenceBuilder lineUpForSinglePixelFarBackBoard( TrajectorySequenceBuilder sequenceBuilder, Vector2d endPosition, double heading) {
        return sequenceBuilder
                .waitSeconds(2)
                .forward(4)
                .waitSeconds(.1)
                .UNSTABLE_addTemporalMarkerOffset(-2, () -> {
                    neoArmWrapper.ClosePos();
                })
                .UNSTABLE_addTemporalMarkerOffset(-1.5, () -> {
                    neoArmWrapper.WristUp();
                    neoArmWrapper.armWristServo.setPosition(NeoArmWrapper.arm_wrist_intake_pos);
                })
                .lineToLinearHeading( new Pose2d(endPosition.getX(), endPosition.getY(), Math.toRadians(heading)));

    }

    protected TrajectorySequenceBuilder getArmReadyForYellowPixelDrop( TrajectorySequenceBuilder sequenceBuilder, int ext, int act, double wrist) {
        return sequenceBuilder.UNSTABLE_addTemporalMarkerOffset(.5, () -> {

            //                   neoArmWrapper.setArmPositions(1000, 650);
            neoArmWrapper.MoveExtensionMotors(ext);
            neoArmWrapper.MoveActuatorMotor(act);
            neoArmWrapper.armWristServo.setPosition(wrist);
        });
    }


    protected TrajectorySequenceBuilder performYellowPixelDrop( TrajectorySequenceBuilder sequenceBuilder, Vector2d location, double heading) {

        return sequenceBuilder.splineTo(location, heading)
                .UNSTABLE_addTemporalMarkerOffset(-.1, () -> {
                    neoArmWrapper.SetWheelSpin(-.6);
                })
                .UNSTABLE_addTemporalMarkerOffset(1.5, () -> {
                    neoArmWrapper.SetWheelSpin(0);
                })
                .waitSeconds(1.5)
                .setReversed(true);
    }

    protected TrajectorySequenceBuilder pickUpOneFarWhitePixels( TrajectorySequenceBuilder sequenceBuilder,
                                                            Vector2d secondLocation, double secondHeading) {


        return sequenceBuilder.UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                    neoArmWrapper.armWristServo.setPosition(NeoArmWrapper.arm_wrist_intake_pos);
                })
                .setReversed(true)
                .waitSeconds(.5)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    neoArmWrapper.WristDown();
                    neoArmWrapper.ClosePos();
                })
                .splineTo(secondLocation, secondHeading)
                .UNSTABLE_addTemporalMarkerOffset(.1, () -> {
                    neoArmWrapper.UpdateIntakePower(1, null);
                })
                .UNSTABLE_addTemporalMarkerOffset(1.1, () -> {
                    neoArmWrapper.OpenPos();
                })
                .UNSTABLE_addTemporalMarkerOffset(1.6, () -> {
                    neoArmWrapper.ClosePos();
                })
                .waitSeconds(2)
                .setReversed(false)
                .UNSTABLE_addTemporalMarkerOffset(2, () -> {
                    neoArmWrapper.SetWheelSpin(-0.2);
                })
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    neoArmWrapper.ClosePos();
                    neoArmWrapper.WristUp();
                })
                .UNSTABLE_addTemporalMarkerOffset(2.5, () -> {
                    neoArmWrapper.UpdateIntakePower(0, null);
                    neoArmWrapper.SetWheelSpin(0);
                });
    }


    protected TrajectorySequenceBuilder pickUpOneFarWhitePixelsMiddle( TrajectorySequenceBuilder sequenceBuilder,
                                                                 Vector2d secondLocation, double secondHeading) {


        return sequenceBuilder.UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                    neoArmWrapper.armWristServo.setPosition(NeoArmWrapper.arm_wrist_intake_pos);
                })
                .setReversed(true)
                .waitSeconds(.5)
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
                    neoArmWrapper.WristDown();
                    neoArmWrapper.ClosePos();
                })
                .splineTo(secondLocation, secondHeading)
                .UNSTABLE_addTemporalMarkerOffset(.1, () -> {
                    neoArmWrapper.UpdateIntakePower(1, null);
                })
                .UNSTABLE_addTemporalMarkerOffset(1.1, () -> {
                    neoArmWrapper.OpenPos();
                })
                .UNSTABLE_addTemporalMarkerOffset(1.6, () -> {
                    neoArmWrapper.ClosePos();
                })
                .waitSeconds(2)
                .setReversed(false)
                .UNSTABLE_addTemporalMarkerOffset(2, () -> {
                    neoArmWrapper.SetWheelSpin(-0.2);
                })
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    neoArmWrapper.ClosePos();
                    neoArmWrapper.WristUp();
                })
                .UNSTABLE_addTemporalMarkerOffset(2.5, () -> {
                    neoArmWrapper.UpdateIntakePower(0, null);
                    neoArmWrapper.SetWheelSpin(0);
                });
    }


    protected TrajectorySequenceBuilder pickUpWhitePixels( TrajectorySequenceBuilder sequenceBuilder,
                 Vector2d firstLocation, double firstHeading, Vector2d secondLocation, double secondHeading) {

        return sequenceBuilder.setReversed(true)
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                    neoArmWrapper.armWristServo.setPosition(NeoArmWrapper.arm_wrist_intake_pos);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.7, () -> {
                    neoArmWrapper.MoveExtensionMotors(-20);
                    neoArmWrapper.MoveActuatorMotor(0);
                })
                .splineTo(firstLocation,firstHeading)
                .waitSeconds(.5)
                .UNSTABLE_addTemporalMarkerOffset(1.5, () -> {
                    neoArmWrapper.SetWheelSpin(1);
                    neoArmWrapper.WristDown();
                    neoArmWrapper.ClosePos();
                })
                .UNSTABLE_addTemporalMarkerOffset(2, () -> {
                    neoArmWrapper.SetWheelSpin(-0.2);
                }) .splineTo(secondLocation, secondHeading)
                .UNSTABLE_addTemporalMarkerOffset(.1, () -> {
                    neoArmWrapper.UpdateIntakePower(1, null);
                })
                .UNSTABLE_addTemporalMarkerOffset(1.1, () -> {
                    neoArmWrapper.OpenPos();
                })
                .UNSTABLE_addTemporalMarkerOffset(2.1, () -> {
                    neoArmWrapper.ClosePos();
                })
                .UNSTABLE_addTemporalMarkerOffset(3.1, () -> {
                    neoArmWrapper.OpenPos();
                })
                .UNSTABLE_addTemporalMarkerOffset(3.6, () -> {
                    neoArmWrapper.ClosePos();
                })
                .waitSeconds(3.7)
                .setReversed(false)
                .UNSTABLE_addTemporalMarkerOffset(1.5, () -> {
                    neoArmWrapper.SetWheelSpin(0);
                    neoArmWrapper.UpdateIntakePower(0, null);
                    neoArmWrapper.ClosePos();
                    neoArmWrapper.WristUp();
                });
    }




    protected TrajectorySequenceBuilder dropOffWhitePixels( TrajectorySequenceBuilder sequenceBuilder,
                                                             Vector2d firstLocation, double firstHeading,
                                                            Vector2d secondLocation, double secondHeading,
                                                            Vector2d thirdLocation

                                                            ) {

        return sequenceBuilder
                .splineTo(firstLocation, firstHeading)
                .waitSeconds(2)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    neoArmWrapper.MoveExtensionMotors(1100);
                    neoArmWrapper.MoveActuatorMotor(1400);
                    neoArmWrapper.armWristServo.setPosition(.1);
                })
                .UNSTABLE_addTemporalMarkerOffset(1.8, () -> {
                    neoArmWrapper.SetWheelSpin(-.6);
                })
                .splineTo(secondLocation, secondHeading)
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                    neoArmWrapper.SetWheelSpin(0);
                })
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                    neoArmWrapper.SetWheelSpin(-1);
                })
                .UNSTABLE_addTemporalMarkerOffset(2, () -> {
                    neoArmWrapper.SetWheelSpin(0);
                })
                .UNSTABLE_addTemporalMarkerOffset(3.3, () -> {
                    neoArmWrapper.MoveExtensionMotors(500);
                })
                .UNSTABLE_addTemporalMarkerOffset(3.8, () -> {
                    neoArmWrapper.armWristServo.setPosition(NeoArmWrapper.arm_wrist_intake_pos);
                })
                .UNSTABLE_addTemporalMarkerOffset(4.5, () -> {
                    neoArmWrapper.MoveActuatorMotor(0);
                })
                .UNSTABLE_addTemporalMarkerOffset(5.5, () -> {
                    neoArmWrapper.MoveExtensionMotors(-20);
                })
                .lineToLinearHeading(new Pose2d(thirdLocation.getX(),thirdLocation.getY(),secondHeading))
                .waitSeconds(3);


//                .waitSeconds(3);
    }



    /*protected TrajectorySequenceBuilder dropOffWhitePixelsFar3( TrajectorySequenceBuilder sequenceBuilder,
                                                            Vector2d firstLocation, double firstHeading,
                                                            Vector2d secondLocation, double secondHeading,
                                                            Vector2d thirdLocation

    ) {

        return sequenceBuilder
                .splineTo(firstLocation, firstHeading)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    neoArmWrapper.MoveExtensionMotors(1100);
                    neoArmWrapper.MoveActuatorMotor(1400);
                    neoArmWrapper.armWristServo.setPosition(.05);
                })
                .UNSTABLE_addTemporalMarkerOffset(1.5, () -> {
                    neoArmWrapper.SetWheelSpin(-.7);
                })
                .lineToLinearHeading(new Pose2d(secondLocation.getX(),secondLocation.getY(), secondHeading))
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                    neoArmWrapper.SetWheelSpin(0);
                })
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                    neoArmWrapper.SetWheelSpin(-1);
                })
                .UNSTABLE_addTemporalMarkerOffset(2, () -> {
                    neoArmWrapper.SetWheelSpin(0);
                })
                .UNSTABLE_addTemporalMarkerOffset(3.3, () -> {
                    neoArmWrapper.MoveExtensionMotors(500);
                })
                .UNSTABLE_addTemporalMarkerOffset(3.8, () -> {
                    neoArmWrapper.armWristServo.setPosition(NeoArmWrapper.arm_wrist_intake_pos);
                })
                .UNSTABLE_addTemporalMarkerOffset(4.5, () -> {
                    neoArmWrapper.MoveActuatorMotor(0);
                })
                .UNSTABLE_addTemporalMarkerOffset(5.5, () -> {
                    neoArmWrapper.MoveExtensionMotors(-20);
                })
                .lineToLinearHeading(new Pose2d(thirdLocation.getX(),thirdLocation.getY(),secondHeading))
                .waitSeconds(3);


//                .waitSeconds(3);
    }*/

    protected TrajectorySequenceBuilder dropOffWhitePixelsRedMiddle(TrajectorySequenceBuilder sequenceBuilder,
                                                                    Vector2d firstLocation, double firstHeading,
                                                                    Vector2d secondLocation, double secondHeading,
                                                                    Vector2d thirdLocation, double thirdHeading,
                                                                    Vector2d fourthLocation

    ) {

        return sequenceBuilder
                .forward(3)
                .waitSeconds(.1)
                .strafeLeft(12)
                .waitSeconds(.1)
                .splineTo(firstLocation, firstHeading)
                .splineTo(secondLocation, secondHeading)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    neoArmWrapper.MoveExtensionMotors(1100);
                    neoArmWrapper.MoveActuatorMotor(1400);
                    neoArmWrapper.armWristServo.setPosition(.05);
                })
                .UNSTABLE_addTemporalMarkerOffset(1.5, () -> {
                    neoArmWrapper.SetWheelSpin(-.7);
                })
                .splineTo(thirdLocation, thirdHeading)
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                    neoArmWrapper.SetWheelSpin(0);
                })
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                    neoArmWrapper.SetWheelSpin(-1);
                })
                .UNSTABLE_addTemporalMarkerOffset(2, () -> {
                    neoArmWrapper.SetWheelSpin(0);
                })
                .UNSTABLE_addTemporalMarkerOffset(3.3, () -> {
                    neoArmWrapper.MoveExtensionMotors(500);
                })
                .UNSTABLE_addTemporalMarkerOffset(3.8, () -> {
                    neoArmWrapper.armWristServo.setPosition(NeoArmWrapper.arm_wrist_intake_pos);
                })
                .UNSTABLE_addTemporalMarkerOffset(4.5, () -> {
                    neoArmWrapper.MoveActuatorMotor(0);
                })
                .UNSTABLE_addTemporalMarkerOffset(5.5, () -> {
                    neoArmWrapper.MoveExtensionMotors(-20);
                })
                .lineToLinearHeading(new Pose2d(fourthLocation.getX(),fourthLocation.getY(),thirdHeading))
                .waitSeconds(3)
                .back(2)
                .waitSeconds(.1);



//                .waitSeconds(3);
    }


    protected TrajectorySequenceBuilder dropOffWhitePixelsBlueMiddle(TrajectorySequenceBuilder sequenceBuilder,
                                                                    Vector2d firstLocation, double firstHeading,
                                                                    Vector2d secondLocation, double secondHeading,
                                                                    Vector2d thirdLocation, double thirdHeading,
                                                                    Vector2d fourthLocation

    ) {

        return sequenceBuilder
                .forward(3)
                .waitSeconds(.1)
                .strafeRight(12)
                .waitSeconds(.1)
                .splineTo(firstLocation, firstHeading)
                .splineTo(secondLocation, secondHeading)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    neoArmWrapper.MoveExtensionMotors(1100);
                    neoArmWrapper.MoveActuatorMotor(1400);
                    neoArmWrapper.armWristServo.setPosition(.1);
                })
                .UNSTABLE_addTemporalMarkerOffset(1.5, () -> {
                    neoArmWrapper.SetWheelSpin(-.6);
                })
                .splineTo(thirdLocation, thirdHeading)
                .waitSeconds(1.5)
                .UNSTABLE_addTemporalMarkerOffset(.5, () -> {
                    neoArmWrapper.SetWheelSpin(0);
                })
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                    neoArmWrapper.SetWheelSpin(-.6);
                })
                .UNSTABLE_addTemporalMarkerOffset(2, () -> {
                    neoArmWrapper.SetWheelSpin(0);
                })
                .UNSTABLE_addTemporalMarkerOffset(3.3, () -> {
                    neoArmWrapper.MoveExtensionMotors(500);
                })
                .UNSTABLE_addTemporalMarkerOffset(3.8, () -> {
                    neoArmWrapper.armWristServo.setPosition(NeoArmWrapper.arm_wrist_intake_pos);
                })
                .UNSTABLE_addTemporalMarkerOffset(4.5, () -> {
                    neoArmWrapper.MoveActuatorMotor(0);
                })
                .UNSTABLE_addTemporalMarkerOffset(5.5, () -> {
                    neoArmWrapper.MoveExtensionMotors(-20);
                })
                .lineToLinearHeading(new Pose2d(fourthLocation.getX(),fourthLocation.getY(),thirdHeading))
                .waitSeconds(3)
                .back(2)
                .waitSeconds(.1);



//                .waitSeconds(3);
    }




    protected TrajectorySequenceBuilder dropOffWhitePixels( TrajectorySequenceBuilder sequenceBuilder,
                                                            Vector2d firstLocation, double firstHeading,
                                                            Vector2d secondLocation, double secondHeading
                                                            ) {

        return sequenceBuilder.splineTo(firstLocation, firstHeading)
                .UNSTABLE_addTemporalMarkerOffset(.5, () -> {
                    neoArmWrapper.MoveExtensionMotors(1100);
                    neoArmWrapper.MoveActuatorMotor(1400);
                    neoArmWrapper.armWristServo.setPosition(.1);
                })
                .splineTo(secondLocation, secondHeading)
                .UNSTABLE_addTemporalMarkerOffset(0.4, () -> {
                    neoArmWrapper.SetWheelSpin(-1);
                })
                .UNSTABLE_addTemporalMarkerOffset(2.6, () -> {
                    neoArmWrapper.SetWheelSpin(0);
                })
                .waitSeconds(2.5)
                .UNSTABLE_addTemporalMarkerOffset(.5, () -> {
                    neoArmWrapper.armWristServo.setPosition(NeoArmWrapper.arm_wrist_intake_pos);
                    neoArmWrapper.MoveActuatorMotor(0);
                })
                .UNSTABLE_addTemporalMarkerOffset(2, () -> {
                    neoArmWrapper.MoveExtensionMotors(-20);
                });
//                .waitSeconds(3);
    }

    protected TrajectorySequenceBuilder park( TrajectorySequenceBuilder sequenceBuilder,
                                                            Pose2d finalLocation

    ) {

        return sequenceBuilder.lineToLinearHeading(finalLocation)
                .waitSeconds(3);
    }









}
