package org.firstinspires.ftc.teamcode.Auto;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.PoseStorage;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.wrappers.RedPropWrapper;

@Autonomous
public class Red_Far extends LinearOpMode {
    Pose2d startPose = new Pose2d(-36,-62, Math.toRadians(-90));

    RedPropWrapper redPropWrapper;

    static int barcodeInt = 2;

    @Override
    public void runOpMode() throws InterruptedException {

        redPropWrapper = new RedPropWrapper(hardwareMap,telemetry);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(startPose);

        TrajectorySequence trajectory1;
        TrajectorySequence trajectory2;
        TrajectorySequence trajectory3;

        trajectory1 = drive.trajectorySequenceBuilder(new Pose2d(-36, -62, Math.toRadians(-90)))
            .setReversed(true)
            .splineTo(new Vector2d(-40, -36), Math.toRadians(135))
            .setReversed(false)
            .waitSeconds(.5)
            .strafeLeft(15)
            .setReversed(true)
            .splineTo(new Vector2d(-60,-12), Math.toRadians(180))
            .setReversed(false)
            .waitSeconds(.5)
            .splineTo(new Vector2d(24, -12), Math.toRadians(0))
            .splineTo(new Vector2d(51, -30), Math.toRadians(0))
            .setReversed(true)
            .waitSeconds(.5)
            .splineTo(new Vector2d(24, -12), Math.toRadians(180))
            .splineTo(new Vector2d(-60, -12), Math.toRadians(180))
            .waitSeconds(.5)
            .setReversed(false)
            .splineTo(new Vector2d(24, -12), Math.toRadians(0))
            .splineTo(new Vector2d(51, -30), Math.toRadians(0))
            .waitSeconds(.5)
            .lineToLinearHeading(new Pose2d(48,-12, Math.toRadians(0)))
            .lineToLinearHeading(new Pose2d(60,-12, Math.toRadians(0)))
            .build();

        trajectory2 = drive.trajectorySequenceBuilder(new Pose2d(-36, -62, Math.toRadians(-90)))
            .lineToLinearHeading(new Pose2d(-36, -32, Math.toRadians(-90)))
            .waitSeconds(.5)
            .lineToLinearHeading(new Pose2d(-50, -32, Math.toRadians(-90)))
            .setReversed(true)
            .splineTo(new Vector2d(-60,-12), Math.toRadians(180))
            .setReversed(false)
            .waitSeconds(.5)
            .splineTo(new Vector2d(24, -12), Math.toRadians(0))
            .splineTo(new Vector2d(51, -36), Math.toRadians(0))
            .setReversed(true)
            .waitSeconds(.5)
            .splineTo(new Vector2d(24, -12), Math.toRadians(180))
            .splineTo(new Vector2d(-60, -12), Math.toRadians(180))
            .waitSeconds(.5)
            .setReversed(false)
            .splineTo(new Vector2d(24, -12), Math.toRadians(0))
            .splineTo(new Vector2d(51, -30), Math.toRadians(0))
            .waitSeconds(.5)
            .lineToLinearHeading(new Pose2d(48,-12, Math.toRadians(0)))
            .lineToLinearHeading(new Pose2d(60,-12, Math.toRadians(0)))
            .build();

        trajectory3 = drive.trajectorySequenceBuilder(new Pose2d(-36, -62, Math.toRadians(-90)))
            .setReversed(true)
            .splineTo(new Vector2d(-32, -36), Math.toRadians(45))
            .setReversed(false)
            .waitSeconds(.5)
            .strafeRight(10)
            .setReversed(true)
            .splineTo(new Vector2d(-60,-12), Math.toRadians(180))
            .setReversed(false)
            .waitSeconds(.5)
            .splineTo(new Vector2d(24, -12), Math.toRadians(0))
            .splineTo(new Vector2d(51, -42), Math.toRadians(0))
            .setReversed(true)
            .waitSeconds(.5)
            .splineTo(new Vector2d(24, -12), Math.toRadians(180))
            .splineTo(new Vector2d(-60, -12), Math.toRadians(180))
            .waitSeconds(.5)
            .setReversed(false)
            .splineTo(new Vector2d(24, -12), Math.toRadians(0))
            .splineTo(new Vector2d(51, -30), Math.toRadians(0))
            .waitSeconds(.5)
            .lineToLinearHeading(new Pose2d(48,-12, Math.toRadians(0)))
            .lineToLinearHeading(new Pose2d(60,-12, Math.toRadians(0)))
            .build();




        waitForStart();
        redPropWrapper.detect();
        barcodeInt = redPropWrapper.getBarcodeInt();

        if (barcodeInt == 1) {
            drive.followTrajectorySequence(trajectory1);
        } else if (barcodeInt == 2) {
            drive.followTrajectorySequence(trajectory2);
        } else {
            drive.followTrajectorySequence(trajectory3);
        }

        PoseStorage.currentPose = drive.getPoseEstimate();

    }


}
