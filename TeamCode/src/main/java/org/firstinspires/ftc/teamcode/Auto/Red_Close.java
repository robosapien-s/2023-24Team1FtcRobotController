package org.firstinspires.ftc.teamcode.Auto;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.wrappers.NeoArmWrapper;

@Autonomous
public class Red_Close extends LinearOpMode {
    Pose2d startPose = new Pose2d(12,-62, Math.toRadians(-90));

    NeoArmWrapper neoArmWrapper;

    static int barcodeInt = 2;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        neoArmWrapper = new NeoArmWrapper(telemetry, hardwareMap, gamepad1, gamepad2);
        neoArmWrapper.ResetMotorPositions();

        drive.setPoseEstimate(startPose);

        TrajectorySequence trajectory1;

        if (barcodeInt == 1) {
            trajectory1 = drive.trajectorySequenceBuilder(new Pose2d(12, -62, Math.toRadians(-90)))
                    .setReversed(true)
                    .splineTo(new Vector2d(6, -36), Math.toRadians(135))
                    .setReversed(false)
                    .waitSeconds(.5)
                    .splineTo(new Vector2d(51, -30), Math.toRadians(0))
                    .waitSeconds(.5)
                    .setReversed(true)
                    .waitSeconds(.5)
                    .splineTo(new Vector2d(24, -12), Math.toRadians(180))
                    .splineTo(new Vector2d(-60, -12), Math.toRadians(180))
                    .waitSeconds(.5)
                    .setReversed(false)
                    .splineTo(new Vector2d(24, -12), Math.toRadians(0))
                    .splineTo(new Vector2d(51, -42), Math.toRadians(0))
                    .waitSeconds(.5)
                    .lineToLinearHeading(new Pose2d(48,-60, Math.toRadians(0)))
                    .lineToLinearHeading(new Pose2d(59,-62, Math.toRadians(0)))
                    .build();
        } else if (barcodeInt == 2) {
            trajectory1 = drive.trajectorySequenceBuilder(new Pose2d(12, -62, Math.toRadians(-90)))
                    .UNSTABLE_addTemporalMarkerOffset(.5, () -> {
                        neoArmWrapper.WristDown();
                    })
                    .lineToLinearHeading(new Pose2d(12, -38, Math.toRadians(-90)))
                    .UNSTABLE_addTemporalMarkerOffset(.1, () -> {
                        neoArmWrapper.OpenPos();
                    })
                    .UNSTABLE_addTemporalMarkerOffset(.3, () -> {
                        neoArmWrapper.WristUp();
                    })
                    .waitSeconds(.5)
                    .UNSTABLE_addTemporalMarkerOffset(.3, () -> {
                        neoArmWrapper.setOuttake();
                        neoArmWrapper.MoveExtensionMotors(250);
                    })
                    .splineTo(new Vector2d(51, -36), Math.toRadians(0))
                    .UNSTABLE_addTemporalMarkerOffset(-.5, () -> {
                        neoArmWrapper.SetWheelSpin(-1);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                        neoArmWrapper.SetWheelSpin(0);
                    })
                    .waitSeconds(1.5)
                    .setReversed(true)
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                        neoArmWrapper.MoveExtensionMotors(0);
                        neoArmWrapper.setIntake();
                    })
                    .splineTo(new Vector2d(24, -12), Math.toRadians(180))
                    .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
                            neoArmWrapper.SetWheelSpin(1);
                            })
                    .UNSTABLE_addTemporalMarkerOffset(1.2, () -> {
                            neoArmWrapper.SetWheelSpin(0);
                            neoArmWrapper.WristDown();
                            })
                    .splineTo(new Vector2d(-58, -12), Math.toRadians(180))
                    .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                        neoArmWrapper.UpdateIntakePower(1);
                        neoArmWrapper.ClosePos();
                    })
                    .UNSTABLE_addTemporalMarkerOffset(2, () -> {
                        neoArmWrapper.OpenPos();
                    })
                    .UNSTABLE_addTemporalMarkerOffset(3, () -> {
                        neoArmWrapper.ClosePos();
                    })
                    .UNSTABLE_addTemporalMarkerOffset(4, () -> {
                        neoArmWrapper.OpenPos();
                        neoArmWrapper.UpdateIntakePower(0);
                    })

                    .waitSeconds(15)
                    .setReversed(false)
                    /*.splineTo(new Vector2d(24, -12), Math.toRadians(0))
                    .splineTo(new Vector2d(51, -42), Math.toRadians(0))
                    .waitSeconds(.5)
                    .lineToLinearHeading(new Pose2d(48,-60, Math.toRadians(0)))
                    .lineToLinearHeading(new Pose2d(59,-62, Math.toRadians(0)))*/
                    .build();
        } else {
            trajectory1 = drive.trajectorySequenceBuilder(new Pose2d(12, -62, Math.toRadians(-90)))
                    .setReversed(true)
                    .splineTo(new Vector2d(18, -36), Math.toRadians(45))
                    .setReversed(false)
                    .waitSeconds(.5)
                    .splineTo(new Vector2d(51, -42), Math.toRadians(0))
                    .waitSeconds(.5)
                    .setReversed(true)
                    .waitSeconds(.5)
                    .splineTo(new Vector2d(24, -12), Math.toRadians(180))
                    .splineTo(new Vector2d(-60, -12), Math.toRadians(180))
                    .waitSeconds(.5)
                    .setReversed(false)
                    .splineTo(new Vector2d(24, -12), Math.toRadians(0))
                    .splineTo(new Vector2d(51, -42), Math.toRadians(0))
                    .waitSeconds(.5)
                    .lineToLinearHeading(new Pose2d(48,-60, Math.toRadians(0)))
                    .lineToLinearHeading(new Pose2d(59,-62, Math.toRadians(0)))
                    .build();
        }


        /*trajectory1 = drive.trajectorySequenceBuilder(new Pose2d(12, -62, Math.toRadians(-90)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                neoArmWrapper.SetWheelSpin(1);
                telemetry.addData("First Marker", "Reached");})
                .UNSTABLE_addTemporalMarkerOffset(5, () -> {
                neoArmWrapper.SetWheelSpin(0);
                telemetry.addData("Second Marker", "Reached");
                telemetry.update();})
                .strafeRight(.1)
                .waitSeconds(10)
                .build();*/


        waitForStart();

        drive.followTrajectorySequence(trajectory1);

    }


}
