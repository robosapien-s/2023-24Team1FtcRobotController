package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.OpModes.InitActuator;
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

import java.net.PortUnreachableException;

public abstract class BaseAutoOp extends LinearOpMode implements ITrajectorySequenceUpdateCallback {


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

        drive.setPoseEstimate(startPose);

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
                     neoArmWrapper.MoveActuatorMotor(-15-InitActuatorPos.actuatorPos);
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
                    neoArmWrapper.MoveActuatorMotor(-15-InitActuatorPos.actuatorPos);
                })

                .UNSTABLE_addTemporalMarkerOffset(1.5, () -> {
                    neoArmWrapper.ResetMotorPositions();
                })
                .lineToLinearHeading( new Pose2d(endPosition.getX(), endPosition.getY(), Math.toRadians(heading)),
                        SampleMecanumDrive.getVelocityConstraint(40,40,DriveConstants.TRACK_WIDTH),
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
                    neoArmWrapper.SetWheelSpin(-1);
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
                    neoArmWrapper.UpdateIntakePower(1);
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
                .UNSTABLE_addTemporalMarkerOffset(2, () -> {
                    neoArmWrapper.UpdateIntakePower(0);
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
                    neoArmWrapper.UpdateIntakePower(1);
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
                    neoArmWrapper.UpdateIntakePower(0);
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
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    neoArmWrapper.MoveExtensionMotors(1100);
                    neoArmWrapper.MoveActuatorMotor(1400);
                    neoArmWrapper.armWristServo.setPosition(.05);
                })
                .UNSTABLE_addTemporalMarkerOffset(1.5, () -> {
                    neoArmWrapper.SetWheelSpin(-1);
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
    protected TrajectorySequenceBuilder dropOffWhitePixels( TrajectorySequenceBuilder sequenceBuilder,
                                                            Vector2d firstLocation, double firstHeading,
                                                            Vector2d secondLocation, double secondHeading
                                                            ) {

        return sequenceBuilder.splineTo(firstLocation, firstHeading)
                .UNSTABLE_addTemporalMarkerOffset(.5, () -> {
                    neoArmWrapper.MoveExtensionMotors(1100);
                    neoArmWrapper.MoveActuatorMotor(1400);
                    neoArmWrapper.armWristServo.setPosition(.05);
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
