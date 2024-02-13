package org.firstinspires.ftc.teamcode.Auto;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
public class Red_Close extends LinearOpMode {
    Pose2d startPose = new Pose2d(12,-62, Math.toRadians(-90));


    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(startPose);

        TrajectorySequence trajectory1 = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(12, -36, Math.toRadians(-90)))
                .waitSeconds(.5)
                .splineTo(new Vector2d(51, -36), Math.toRadians(0))
                .waitSeconds(.5)
                .lineToLinearHeading(new Pose2d(-60,-36, Math.toRadians(0)))
                .waitSeconds(.5)
                .lineToLinearHeading(new Pose2d(51,-36, Math.toRadians(0)))
                .waitSeconds(.5)
                .lineToLinearHeading(new Pose2d(48,-62, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(60,-62, Math.toRadians(0)))
                .build();

        waitForStart();

        if (!isStopRequested()) {
            drive.followTrajectorySequence(trajectory1);
        }

    }


}
