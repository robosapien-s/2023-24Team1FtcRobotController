package org.firstinspires.ftc.teamcode.Auto;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.wrappers.NeoArmWrapper;

@Autonomous
public class EmptyTrajectory extends LinearOpMode {
    Pose2d startPose = new Pose2d(-36,-62, Math.toRadians(-90));

    static int barcodeInt = 3;

    NeoArmWrapper neoArmWrapper;


    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(startPose);

        TrajectorySequence trajectory1;

        neoArmWrapper = new NeoArmWrapper(telemetry, hardwareMap, gamepad1, gamepad2,true);
        neoArmWrapper.ResetMotorPositions();

        neoArmWrapper.ActivateLoop();

        trajectory1 = drive.trajectorySequenceBuilder(new Pose2d(-36, -62, Math.toRadians(-90)))
            .UNSTABLE_addTemporalMarkerOffset(.3, () -> {
                neoArmWrapper.SetLinearExtensionPos(250);
            })
            .back(1)
            .waitSeconds(100)
            .forward(1)
            .build();

        waitForStart();

        drive.followTrajectorySequence(trajectory1);

    }


}
