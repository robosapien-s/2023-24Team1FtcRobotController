package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.ITrajectorySequenceUpdateCallback;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.wrappers.NeoArmWrapper;
import org.firstinspires.ftc.teamcode.wrappers.RedPropWrapper;

public abstract class BaseAutoOp extends LinearOpMode implements ITrajectorySequenceUpdateCallback {


    Pose2d startPose = new Pose2d(15,-62, Math.toRadians(-90));

    RedPropWrapper redPropWrapper;

    NeoArmWrapper neoArmWrapper;

    SampleMecanumDrive drive = null;

    static int barcodeInt;


    public void initialize(Pose2d inStartPose, int inBarcodeInt) {
        startPose = inStartPose;

        redPropWrapper = new RedPropWrapper(hardwareMap,telemetry);

        drive = new SampleMecanumDrive(hardwareMap, this);

        neoArmWrapper = new NeoArmWrapper(telemetry, hardwareMap, gamepad1, gamepad2, true);
        neoArmWrapper.ResetMotorPositions();

        drive.setPoseEstimate(startPose);

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
                 .splineTo(endPosition,heading).setReversed(false)
                 .UNSTABLE_addTemporalMarkerOffset(.1, () -> {
            neoArmWrapper.ClosePos();
        })
                .UNSTABLE_addTemporalMarkerOffset(.3, () -> {
                    neoArmWrapper.WristUp();
                })
                .waitSeconds(1);
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


    protected TrajectorySequenceBuilder pickUpWhitePixels( TrajectorySequenceBuilder sequenceBuilder,
                 Vector2d firstLocation, double firstHeading, Vector2d secondLocation, double secondHeading) {

        return sequenceBuilder.UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
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
                                                            Pose2d finalLocation

                                                            ) {

        return sequenceBuilder.splineTo(firstLocation, firstHeading)
                .UNSTABLE_addTemporalMarkerOffset(.5, () -> {
                    neoArmWrapper.MoveExtensionMotors(1100);
                    neoArmWrapper.MoveActuatorMotor(1400);
                    neoArmWrapper.armWristServo.setPosition(.05);
                })
                .splineTo(secondLocation, secondHeading)
                .UNSTABLE_addTemporalMarkerOffset(-.4, () -> {
                    neoArmWrapper.SetWheelSpin(-1);
                })
                .UNSTABLE_addTemporalMarkerOffset(2, () -> {
                    neoArmWrapper.SetWheelSpin(0);
                })
                .waitSeconds(2.5)
                .UNSTABLE_addTemporalMarkerOffset(.5, () -> {
                    neoArmWrapper.armWristServo.setPosition(NeoArmWrapper.arm_wrist_intake_pos);
                    neoArmWrapper.MoveActuatorMotor(0);
                })
                .UNSTABLE_addTemporalMarkerOffset(2, () -> {
                    neoArmWrapper.MoveExtensionMotors(-20);
                })
                .lineToLinearHeading(finalLocation)
                .waitSeconds(3);
    }







}
