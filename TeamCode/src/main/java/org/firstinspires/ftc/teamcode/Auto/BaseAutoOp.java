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
        telemetry.addData("update", System.currentTimeMillis());
        telemetry.update();
        neoArmWrapper.UpdateExtensionPlusInput(null, 300, 300, null, imu);
    }

    protected TrajectorySequenceBuilder setInitialPose(Pose2d pose2d) {
        return drive.trajectorySequenceBuilder(pose2d);
    }

    protected TrajectorySequenceBuilder getArmReady( TrajectorySequenceBuilder sequenceBuilder) {
        return sequenceBuilder.setReversed(true)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    neoArmWrapper.closeLeftPixelHolder();
                    neoArmWrapper.closeRightPixelHolder();
                    neoArmWrapper.WristUp();
                });
    }

    protected TrajectorySequenceBuilder dropPurplePixel( TrajectorySequenceBuilder sequenceBuilder, Vector2d endPosition, double heading) {
         return sequenceBuilder

                 .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                     neoArmWrapper.MoveActuatorMotor(-10);
                 })


//                 .UNSTABLE_addTemporalMarkerOffset(1.5, () -> {
//                     neoArmWrapper.ResetMotorPositions();
//                 })

                 .splineTo(
                         endPosition,
                         heading,
                         SampleMecanumDrive.getVelocityConstraint(35,35,DriveConstants.TRACK_WIDTH),
                         SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)).setReversed(false)
                 .UNSTABLE_addTemporalMarkerOffset(-.3, () -> {
            neoArmWrapper.WristDown();
        })


                .UNSTABLE_addTemporalMarkerOffset(.3, () -> {
                    neoArmWrapper.WristUp();
                    neoArmWrapper.closeLeftPixelHolder();
                    neoArmWrapper.closeRightPixelHolder();
                    neoArmWrapper.setOuttakeNew(NeoArmWrapper.EPixelHolderLocation.SINGLE);
                })
                .waitSeconds(1);
    }


    protected TrajectorySequenceBuilder dropPurplePixelLine( TrajectorySequenceBuilder sequenceBuilder, Vector2d endPosition, double heading) {
        return dropPurplePixelLine(sequenceBuilder, endPosition, heading, true);
    }

    protected TrajectorySequenceBuilder dropPurplePixelLine( TrajectorySequenceBuilder sequenceBuilder, Vector2d endPosition, double heading, boolean setOuttake) {
        return sequenceBuilder

                .UNSTABLE_addTemporalMarkerOffset(0, () -> {

                    //neoArmWrapper.ResetMotorPositions();
                    //neoArmWrapper.SetLinearActuatorTask(-InitActuatorPos.actuatorPos);
                    neoArmWrapper.MoveActuatorMotor(-10);
                })

                .UNSTABLE_addTemporalMarkerOffset(1.5, () -> {

                    neoArmWrapper.ResetMotorPositionsWithoutZero();
                })

                .lineToLinearHeading(new Pose2d(endPosition.getX(), endPosition.getY(), heading),
                        SampleMecanumDrive.getVelocityConstraint(25,25,DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)).setReversed(false)
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
                    neoArmWrapper.WristDown();

                })

                .UNSTABLE_addTemporalMarkerOffset(0, () -> {neoArmWrapper.PickupStack5Turn(0);})

                .UNSTABLE_addTemporalMarkerOffset(.8, () -> {
                    neoArmWrapper.WristUp();
                    neoArmWrapper.closeLeftPixelHolder();
                    neoArmWrapper.closeRightPixelHolder();
                    if(setOuttake) {
                        neoArmWrapper.setOuttakeNewWithAct(NeoArmWrapper.EPixelHolderLocation.SINGLE,1000);
                    }
                })

                .waitSeconds(1.2);
    }


    protected TrajectorySequenceBuilder getArmReadyFar( TrajectorySequenceBuilder sequenceBuilder) {
        return sequenceBuilder
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    neoArmWrapper.closeLeftPixelHolder();
                    neoArmWrapper.closeRightPixelHolder();
                    neoArmWrapper.WristUp();
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
//                .waitSeconds(2)
//                .forward(4)
                .waitSeconds(1)
//                .UNSTABLE_addTemporalMarkerOffset(-2, () -> {
//                    neoArmWrapper.ClosePos();
//                })
                .UNSTABLE_addTemporalMarkerOffset(-1.5, () -> {
                    neoArmWrapper.WristUp();
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

        return sequenceBuilder.lineToLinearHeading(new Pose2d(location.getX(),location.getY(),heading))
                .UNSTABLE_addTemporalMarkerOffset(.2, () -> {
                    neoArmWrapper.setActuatorPosition(800);
                    neoArmWrapper.setExtPosition(700);
                })
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {

                    neoArmWrapper.openLeftPixelHolder();
                    neoArmWrapper.openRightPixelHolder();
                })
                .UNSTABLE_addTemporalMarkerOffset(1.8, () -> {
                    neoArmWrapper.closeRightPixelHolder();
                    neoArmWrapper.closeLeftPixelHolder();
                    //neoArmWrapper.setActuatorPosition(0);
                    neoArmWrapper.setExtPosition(250);
                })
                .waitSeconds(2.2)
                .setReversed(true);
    }

    protected TrajectorySequenceBuilder performYellowPixelDrop( TrajectorySequenceBuilder sequenceBuilder, Vector2d location, double heading, int actuatorPosition) {

        return sequenceBuilder.lineToLinearHeading(new Pose2d(location.getX(),location.getY(),heading))
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                    neoArmWrapper.setExtPosition(700);
                    neoArmWrapper.setActuatorPosition(actuatorPosition);
                })
                .UNSTABLE_addTemporalMarkerOffset(1.5, () -> {

                    neoArmWrapper.openLeftPixelHolder();
                    neoArmWrapper.openRightPixelHolder();
                })
                .UNSTABLE_addTemporalMarkerOffset(2.5, () -> {
                    neoArmWrapper.closeRightPixelHolder();
                    neoArmWrapper.closeLeftPixelHolder();
                    //neoArmWrapper.setActuatorPosition(0)
                    neoArmWrapper.setExtPosition(250);

                })
                .waitSeconds(2.5)
                .setReversed(true);
    }
    protected TrajectorySequenceBuilder performYellowPixelDrop( TrajectorySequenceBuilder sequenceBuilder, Vector2d location, double heading, int actuatorPosition, int extensionPosition) {

        return sequenceBuilder.lineToLinearHeading(new Pose2d(location.getX(),location.getY(),heading))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    neoArmWrapper.setExtPosition(actuatorPosition);
                    neoArmWrapper.setExtPosition(extensionPosition);
                })
                .UNSTABLE_addTemporalMarkerOffset(.5, () -> {

                    neoArmWrapper.openLeftPixelHolder();
                    neoArmWrapper.openRightPixelHolder();
                })
                .UNSTABLE_addTemporalMarkerOffset(1.2, () -> {
                    neoArmWrapper.closeRightPixelHolder();
                    neoArmWrapper.closeLeftPixelHolder();
                    //neoArmWrapper.setActuatorPosition(0);
                    neoArmWrapper.setExtPosition(250);
                })
                .waitSeconds(2.5)
                .setReversed(true);
    }

    protected TrajectorySequenceBuilder pickUpOneFarWhitePixels( TrajectorySequenceBuilder sequenceBuilder,
                                                            Vector2d secondLocation, double secondHeading) {

        return sequenceBuilder.setReversed(true)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    //neoArmWrapper.armWristServo.setPosition(NeoArmWrapper.arm_wrist_intake_pos);
                    //neoArmWrapper.setIntakeNew();
                    neoArmWrapper.WristDown();
                })
//                .UNSTABLE_addTemporalMarkerOffset(0.7, () -> {
//                    neoArmWrapper.MoveExtensionMotors(-20);
//                    neoArmWrapper.MoveActuatorMotor(0);
//                })

                .waitSeconds(.1)
                .lineToLinearHeading(new Pose2d(secondLocation.getX(),secondLocation.getY(), secondHeading), SampleMecanumDrive.getVelocityConstraint(20,20,DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(20))
                .UNSTABLE_addTemporalMarkerOffset(.1, () -> {
                    neoArmWrapper.UpdateIntakePower(1, null);
                })
                .UNSTABLE_addTemporalMarkerOffset(.7, () -> {
                    neoArmWrapper.PickupStack5Turn(.5f);
                })
                .waitSeconds(1.8)
                .setReversed(false)
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                    neoArmWrapper.PickupStack5Turn(0);
                })
                .UNSTABLE_addTemporalMarkerOffset(1.5, () -> {
                    neoArmWrapper.UpdateIntakePower(0, null);
                    neoArmWrapper.WristUp();
                });


//
//
//        return sequenceBuilder.UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
//                    neoArmWrapper.armWristServo.setPosition(NeoArmWrapper.arm_wrist_intake_pos);
//                })
//                .setReversed(true)
//                .waitSeconds(.5)
//                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
//                    neoArmWrapper.WristDown();
//                    neoArmWrapper.ClosePos();
//                })
//                .splineTo(secondLocation, secondHeading)
//                .UNSTABLE_addTemporalMarkerOffset(.1, () -> {
//                    neoArmWrapper.UpdateIntakePower(1, null);
//                })
//                .UNSTABLE_addTemporalMarkerOffset(1.1, () -> {
//                    neoArmWrapper.OpenPos();
//                })
//                .UNSTABLE_addTemporalMarkerOffset(1.6, () -> {
//                    neoArmWrapper.ClosePos();
//                })
//                .waitSeconds(2)
//                .setReversed(false)
//                .UNSTABLE_addTemporalMarkerOffset(2, () -> {
//                    neoArmWrapper.SetWheelSpin(-0.2);
//                })
//                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
//                    neoArmWrapper.ClosePos();
//                    neoArmWrapper.WristUp();
//                })
//                .UNSTABLE_addTemporalMarkerOffset(2.5, () -> {
//                    neoArmWrapper.UpdateIntakePower(0, null);
//                    neoArmWrapper.SetWheelSpin(0);
//                });
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

                .UNSTABLE_addTemporalMarkerOffset(.3, () -> {

                    neoArmWrapper.setActuatorPosition(250);
                    neoArmWrapper.setExtPosition(250);
                })

                .UNSTABLE_addTemporalMarkerOffset(1.3, () -> {
                    //neoArmWrapper.armWristServo.setPosition(NeoArmWrapper.arm_wrist_intake_pos);
                    neoArmWrapper.setIntakeNew();
                })
//                .UNSTABLE_addTemporalMarkerOffset(0.7, () -> {
//                    neoArmWrapper.MoveExtensionMotors(-20);
//                    neoArmWrapper.MoveActuatorMotor(0);
//                })
                .splineTo(firstLocation,firstHeading)

               .waitSeconds(1.3)
                .UNSTABLE_addTemporalMarkerOffset(1.5, () -> {
                   // neoArmWrapper.SetWheelSpin(1);
                    neoArmWrapper.WristDown();
                   // neoArmWrapper.ClosePos();
                })
                .UNSTABLE_addTemporalMarkerOffset(2, () -> {
                   // neoArmWrapper.SetWheelSpin(-0.2);
                })
                .splineTo(new Vector2d(secondLocation.getX()+10, secondLocation.getY()), secondHeading, SampleMecanumDrive.getVelocityConstraint(40,40,DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(40))

                //.splineTo(secondLocation, secondHeading, SampleMecanumDrive.getVelocityConstraint(35,35,DriveConstants.TRACK_WIDTH),
                //SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineToLinearHeading(new Pose2d(secondLocation.getX(),secondLocation.getY()), SampleMecanumDrive.getVelocityConstraint(15,15,DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(15))
                .UNSTABLE_addTemporalMarkerOffset(.1, () -> {
                    neoArmWrapper.UpdateIntakePower(1, null);
                })
                .UNSTABLE_addTemporalMarkerOffset(.7, () -> {
                    neoArmWrapper.PickupStack5Turn(.5f);
                })
                .UNSTABLE_addTemporalMarkerOffset(1.5, () -> {
                    neoArmWrapper.PickupStack5Turn(1);
                })
                .waitSeconds(2.2)
                .setReversed(false)
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                    neoArmWrapper.PickupStack5Turn(0);
                })
                .UNSTABLE_addTemporalMarkerOffset(2.5, () -> {
                    neoArmWrapper.UpdateIntakePower(0, null);
                    neoArmWrapper.WristUp();
                }).waitSeconds(1.2);
    }


    protected TrajectorySequenceBuilder pickUpWhitePixelsFar( TrajectorySequenceBuilder sequenceBuilder,
                                                           Vector2d firstLocation, double firstHeading, Vector2d secondLocation, double secondHeading) {

        return sequenceBuilder.setReversed(true)
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                    //neoArmWrapper.armWristServo.setPosition(NeoArmWrapper.arm_wrist_intake_pos);
                    neoArmWrapper.setIntakeNew();
                })
//                .UNSTABLE_addTemporalMarkerOffset(0.7, () -> {
//                    neoArmWrapper.MoveExtensionMotors(-20);
//                    neoArmWrapper.MoveActuatorMotor(0);
//                })
                .splineTo(firstLocation,firstHeading)


                .waitSeconds(.1)
                .UNSTABLE_addTemporalMarkerOffset(1.5, () -> {
                    // neoArmWrapper.SetWheelSpin(1);
                    neoArmWrapper.WristDown();
                    // neoArmWrapper.ClosePos();
                })
                .UNSTABLE_addTemporalMarkerOffset(2, () -> {
                    // neoArmWrapper.SetWheelSpin(-0.2);
                })
                .splineTo(new Vector2d(secondLocation.getX()+20, secondLocation.getY()), firstHeading)

                .lineToLinearHeading(new Pose2d(secondLocation.getX()+19,secondLocation.getY(), 70), SampleMecanumDrive.getVelocityConstraint(20,20,DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(20));

//        .lineToLinearHeading(new Pose2d(secondLocation.getX(),secondLocation.getY(), secondHeading), SampleMecanumDrive.getVelocityConstraint(20,20,DriveConstants.TRACK_WIDTH),
//                SampleMecanumDrive.getAccelerationConstraint(20));

//                .lineToLinearHeading(new Pose2d(secondLocation.getX(),secondLocation.getY()), SampleMecanumDrive.getVelocityConstraint(20,20,DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(20))
//                .UNSTABLE_addTemporalMarkerOffset(.1, () -> {
//                    neoArmWrapper.UpdateIntakePower(1, null);
//                })
//                .UNSTABLE_addTemporalMarkerOffset(.7, () -> {
//                    neoArmWrapper.PickupStack5Turn(.5f);
//                })
//                .UNSTABLE_addTemporalMarkerOffset(1.5, () -> {
//                    neoArmWrapper.PickupStack5Turn(1);
//                })
//                .waitSeconds(2.2)
//                .setReversed(false)
//                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
//                    neoArmWrapper.PickupStack5Turn(0);
//                })
//                .UNSTABLE_addTemporalMarkerOffset(1.5, () -> {
//                    neoArmWrapper.UpdateIntakePower(0, null);
//                    neoArmWrapper.WristUp();
//                });
    }




    protected TrajectorySequenceBuilder dropOffWhitePixels( TrajectorySequenceBuilder sequenceBuilder,
                                                             Vector2d firstLocation, double firstHeading,
                                                            Vector2d secondLocation, double secondHeading,
                                                            Vector2d thirdLocation//, double actPos, double extPos

                                                            ) {

        return sequenceBuilder
                .splineTo(firstLocation, firstHeading)
                //.waitSeconds(2)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    neoArmWrapper.setOuttakeNew(NeoArmWrapper.EPixelHolderLocation.SINGLE);
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
                    neoArmWrapper.MoveActuatorMotor(1900);
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
                    neoArmWrapper.MoveActuatorMotor(1900);
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
                                                            Vector2d secondLocation, double secondHeading,
                                                            double actPos, double extPos, NeoArmWrapper.EPixelHolderLocation pixelLocation
                                                            ) {

        return sequenceBuilder.splineTo(firstLocation, firstHeading)
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
                    neoArmWrapper.setOuttakeNew(pixelLocation);
                    neoArmWrapper.setArmPositions(actPos,extPos);
                })

                .lineToLinearHeading(new Pose2d(secondLocation.getX(), secondLocation.getY(), secondHeading))
                .UNSTABLE_addTemporalMarkerOffset(0.4, () -> {
                    //neoArmWrapper.SetWheelSpin(-1);
                    neoArmWrapper.openLeftPixelHolder();
                    neoArmWrapper.openRightPixelHolder();

                })
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                    neoArmWrapper.closeLeftPixelHolder();
                    neoArmWrapper.closeRightPixelHolder();
                })
                .waitSeconds(.5)
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                    neoArmWrapper.setIntakeNew();
                })
                .waitSeconds(1);
    }

    protected TrajectorySequenceBuilder park( TrajectorySequenceBuilder sequenceBuilder,
                                                            Pose2d finalLocation

    ) {

        return sequenceBuilder.lineToLinearHeading(finalLocation)
                .waitSeconds(3);
    }









}
