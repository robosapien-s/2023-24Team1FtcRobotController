package org.firstinspires.ftc.teamcode.Auto;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.PoseStorage;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.wrappers.NeoArmWrapper;
import org.firstinspires.ftc.teamcode.wrappers.OpenCvDetection;

@Autonomous
public class Red_Close extends LinearOpMode {
    Pose2d startPose = new Pose2d(15,-62, Math.toRadians(-90));

    NeoArmWrapper neoArmWrapper;

    OpenCvDetection OpenCVWrapper;

    boolean red = true;

    static int barcodeInt = 2;

    @Override
    public void runOpMode() throws InterruptedException {
        OpenCVWrapper = new OpenCvDetection(telemetry, hardwareMap);

        OpenCVWrapper.initColor(red);

        OpenCVWrapper.init();

        barcodeInt = OpenCVWrapper.barcodeInt;

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        neoArmWrapper = new NeoArmWrapper(telemetry, hardwareMap, gamepad1, gamepad2);
        neoArmWrapper.ResetMotorPositions();

        neoArmWrapper.ActivateLoop();

        drive.setPoseEstimate(startPose);

        TrajectorySequence trajectory1;

        if (barcodeInt == 1) {
            trajectory1 = drive.trajectorySequenceBuilder(new Pose2d(12, -62, Math.toRadians(-90)))
                    .setReversed(true)
                    .UNSTABLE_addTemporalMarkerOffset(.5, () -> {
                        neoArmWrapper.WristDown();
                    })
                    .splineTo(new Vector2d(6, -40), Math.toRadians(135))
                    .setReversed(false)
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                        neoArmWrapper.OpenPos();
                    })
                    .UNSTABLE_addTemporalMarkerOffset(.3, () -> {
                        neoArmWrapper.WristUp();
                    })
                    .waitSeconds(1)
                    .UNSTABLE_addTemporalMarkerOffset(.5, () -> {
                        neoArmWrapper.MoveExtensionMotors(250);
                        neoArmWrapper.MoveActuatorMotor(1000);
                    })
                    .splineTo(new Vector2d(51, -30), Math.toRadians(0))
                    .UNSTABLE_addTemporalMarkerOffset(-.7, () -> {
                        neoArmWrapper.SetWheelSpin(-1);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(1.5, () -> {
                        neoArmWrapper.SetWheelSpin(0);
                    })
                    .waitSeconds(1.5)
                    .setReversed(true)
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                        neoArmWrapper.MoveExtensionMotors(0);
                        neoArmWrapper.MoveActuatorMotor(0);
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
                    .splineTo(new Vector2d(-57.5, -12), Math.toRadians(180))
                    .UNSTABLE_addTemporalMarkerOffset(.1, () -> {
                        neoArmWrapper.UpdateIntakePower(1);
                        neoArmWrapper.ClosePos();
                    })
                    .UNSTABLE_addTemporalMarkerOffset(1.1, () -> {
                        neoArmWrapper.OpenPos();
                    })
                    .UNSTABLE_addTemporalMarkerOffset(2.1, () -> {
                        neoArmWrapper.ClosePos();
                    })
                    .UNSTABLE_addTemporalMarkerOffset(3.6, () -> {
                        neoArmWrapper.OpenPos();
                    })
                    .waitSeconds(3.7)
                    .setReversed(false)
                    .UNSTABLE_addTemporalMarkerOffset(1.5, () -> {
                        neoArmWrapper.UpdateIntakePower(0);
                        neoArmWrapper.ClosePos();
                        neoArmWrapper.WristUp();
                    })
                    .splineTo(new Vector2d(24, -12), Math.toRadians(0))
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                        neoArmWrapper.MoveExtensionMotors(250);
                        neoArmWrapper.MoveActuatorMotor(1000);
                    })
                    .splineTo(new Vector2d(51, -42), Math.toRadians(0))
                    .UNSTABLE_addTemporalMarkerOffset(-.7, () -> {
                        neoArmWrapper.SetWheelSpin(-1);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(2, () -> {
                        neoArmWrapper.SetWheelSpin(0);
                    })
                    .waitSeconds(2.5)
                    .UNSTABLE_addTemporalMarkerOffset(.5, () -> {
                        neoArmWrapper.MoveActuatorMotor(0);
                        neoArmWrapper.MoveExtensionMotors(0);
                    })
                    .lineToLinearHeading(new Pose2d(48,-60, Math.toRadians(90)))

                    .lineToLinearHeading(new Pose2d(59,-62, Math.toRadians(90)))
                    .build();

            PoseStorage.currentPose = drive.getPoseEstimate();

        } else if (barcodeInt == 2) {
            trajectory1 = drive.trajectorySequenceBuilder(new Pose2d(12, -62, Math.toRadians(-90)))
                    .UNSTABLE_addTemporalMarkerOffset(.5, () -> {
                        neoArmWrapper.WristDown();
                    })
                    .lineToLinearHeading(new Pose2d(12, -38, Math.toRadians(-90)))
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                        neoArmWrapper.OpenPos();
                    })
                    .UNSTABLE_addTemporalMarkerOffset(.3, () -> {
                        neoArmWrapper.WristUp();
                    })
                    .waitSeconds(1)
                    .UNSTABLE_addTemporalMarkerOffset(.5, () -> {
                        neoArmWrapper.MoveExtensionMotors(250);
                        neoArmWrapper.MoveActuatorMotor(1000);
                    })
                    .splineTo(new Vector2d(51, -36), Math.toRadians(0))
                    .UNSTABLE_addTemporalMarkerOffset(-.7, () -> {
                        neoArmWrapper.SetWheelSpin(-1);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(1.5, () -> {
                        neoArmWrapper.SetWheelSpin(0);
                    })
                    .waitSeconds(1.5)
                    .setReversed(true)
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                        neoArmWrapper.MoveExtensionMotors(0);
                        neoArmWrapper.MoveActuatorMotor(0);
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
                    .splineTo(new Vector2d(-57.5, -12), Math.toRadians(180))
                    .UNSTABLE_addTemporalMarkerOffset(.1, () -> {
                        neoArmWrapper.UpdateIntakePower(1);
                        neoArmWrapper.ClosePos();
                    })
                    .UNSTABLE_addTemporalMarkerOffset(1.1, () -> {
                        neoArmWrapper.OpenPos();
                    })
                    .UNSTABLE_addTemporalMarkerOffset(2.1, () -> {
                        neoArmWrapper.ClosePos();
                    })
                    .UNSTABLE_addTemporalMarkerOffset(3.6, () -> {
                        neoArmWrapper.OpenPos();
                    })
                    .waitSeconds(3.7)
                    .setReversed(false)
                    .UNSTABLE_addTemporalMarkerOffset(1.5, () -> {
                        neoArmWrapper.UpdateIntakePower(0);
                        neoArmWrapper.ClosePos();
                        neoArmWrapper.WristUp();
                    })
                    .splineTo(new Vector2d(24, -12), Math.toRadians(0))
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                        neoArmWrapper.MoveExtensionMotors(250);
                        neoArmWrapper.MoveActuatorMotor(1000);
                    })
                    .splineTo(new Vector2d(51, -42), Math.toRadians(0))
                    .UNSTABLE_addTemporalMarkerOffset(-.7, () -> {
                        neoArmWrapper.SetWheelSpin(-1);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(2, () -> {
                        neoArmWrapper.SetWheelSpin(0);
                    })
                    .waitSeconds(2.5)
                    .UNSTABLE_addTemporalMarkerOffset(.5, () -> {
                        neoArmWrapper.MoveActuatorMotor(0);
                        neoArmWrapper.MoveExtensionMotors(0);
                    })
                    .lineToLinearHeading(new Pose2d(48,-60, Math.toRadians(90)))

                    .lineToLinearHeading(new Pose2d(59,-62, Math.toRadians(90)))
                    .build();

            PoseStorage.currentPose = drive.getPoseEstimate();
        } else {
            trajectory1 = drive.trajectorySequenceBuilder(new Pose2d(12, -62, Math.toRadians(-90)))
                    .setReversed(true)
                    .UNSTABLE_addTemporalMarkerOffset(.5, () -> {
                        neoArmWrapper.WristDown();
                    })
                    .splineTo(new Vector2d(18, -40), Math.toRadians(135))
                    .setReversed(false)
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                        neoArmWrapper.OpenPos();
                    })
                    .UNSTABLE_addTemporalMarkerOffset(.3, () -> {
                        neoArmWrapper.WristUp();
                    })
                    .waitSeconds(1)
                    .UNSTABLE_addTemporalMarkerOffset(.5, () -> {
                        neoArmWrapper.MoveExtensionMotors(250);
                        neoArmWrapper.MoveActuatorMotor(1000);
                    })
                    .splineTo(new Vector2d(51, -42), Math.toRadians(0))
                    .UNSTABLE_addTemporalMarkerOffset(-.7, () -> {
                        neoArmWrapper.SetWheelSpin(-1);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(1.5, () -> {
                        neoArmWrapper.SetWheelSpin(0);
                    })
                    .waitSeconds(1.5)
                    .setReversed(true)
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                        neoArmWrapper.MoveExtensionMotors(0);
                        neoArmWrapper.MoveActuatorMotor(0);
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
                    .splineTo(new Vector2d(-57.5, -12), Math.toRadians(180))
                    .UNSTABLE_addTemporalMarkerOffset(.1, () -> {
                        neoArmWrapper.UpdateIntakePower(1);
                        neoArmWrapper.ClosePos();
                    })
                    .UNSTABLE_addTemporalMarkerOffset(1.1, () -> {
                        neoArmWrapper.OpenPos();
                    })
                    .UNSTABLE_addTemporalMarkerOffset(2.1, () -> {
                        neoArmWrapper.ClosePos();
                    })
                    .UNSTABLE_addTemporalMarkerOffset(3.6, () -> {
                        neoArmWrapper.OpenPos();
                    })
                    .waitSeconds(3.7)
                    .setReversed(false)
                    .UNSTABLE_addTemporalMarkerOffset(1.5, () -> {
                        neoArmWrapper.UpdateIntakePower(0);
                        neoArmWrapper.ClosePos();
                        neoArmWrapper.WristUp();
                    })
                    .splineTo(new Vector2d(24, -12), Math.toRadians(0))
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                        neoArmWrapper.MoveExtensionMotors(250);
                        neoArmWrapper.MoveActuatorMotor(1000);
                    })
                    .splineTo(new Vector2d(51, -42), Math.toRadians(0))
                    .UNSTABLE_addTemporalMarkerOffset(-.7, () -> {
                        neoArmWrapper.SetWheelSpin(-1);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(2, () -> {
                        neoArmWrapper.SetWheelSpin(0);
                    })
                    .waitSeconds(2.5)
                    .UNSTABLE_addTemporalMarkerOffset(.5, () -> {
                        neoArmWrapper.MoveActuatorMotor(0);
                        neoArmWrapper.MoveExtensionMotors(0);
                    })
                    .lineToLinearHeading(new Pose2d(48,-60, Math.toRadians(90)))

                    .lineToLinearHeading(new Pose2d(59,-62, Math.toRadians(90)))
                    .build();

            PoseStorage.currentPose = drive.getPoseEstimate();

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
        neoArmWrapper.DeactivateLoop();
    }


}
