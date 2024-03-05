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
import org.firstinspires.ftc.teamcode.wrappers.RedPropWrapper;

import java.util.Timer;
import java.util.TimerTask;

@Autonomous
public class Red_Close extends LinearOpMode {
    Pose2d startPose = new Pose2d(15,-62, Math.toRadians(-90));

    RedPropWrapper redPropWrapper;

    NeoArmWrapper neoArmWrapper;

    static int barcodeInt;

    @Override
    public void runOpMode() throws InterruptedException {

        redPropWrapper = new RedPropWrapper(hardwareMap,telemetry);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        neoArmWrapper = new NeoArmWrapper(telemetry, hardwareMap, gamepad1, gamepad2, true);
        neoArmWrapper.ResetMotorPositions();

        drive.setPoseEstimate(startPose);

        TrajectorySequence trajectory1;
        TrajectorySequence trajectory2;
        TrajectorySequence trajectory3;


        trajectory1 = drive.trajectorySequenceBuilder(new Pose2d(12, -62, Math.toRadians(-90)))
                .setReversed(true)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    neoArmWrapper.OpenPos();
                    neoArmWrapper.WristDown();

                })
                .splineTo(new Vector2d(6, -42),Math.toRadians(135))
                .setReversed(false)
                .UNSTABLE_addTemporalMarkerOffset(.1, () -> {
                    neoArmWrapper.ClosePos();
                })
                .UNSTABLE_addTemporalMarkerOffset(.3, () -> {
                    neoArmWrapper.WristUp();
                })
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(.5, () -> {

                    //                   neoArmWrapper.setArmPositions(1000, 650);
                    neoArmWrapper.MoveExtensionMotors(852);
                    neoArmWrapper.MoveActuatorMotor(1763);
                    neoArmWrapper.armWristServo.setPosition(.1);
                })
                .splineTo(new Vector2d(50, -30), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(-.1, () -> {
                    neoArmWrapper.SetWheelSpin(-1);
                })
                .UNSTABLE_addTemporalMarkerOffset(1.5, () -> {
                    neoArmWrapper.SetWheelSpin(0);
                })
                .waitSeconds(1.5)
                .setReversed(true)
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    neoArmWrapper.armWristServo.setPosition(NeoArmWrapper.arm_wrist_intake_pos);
                })
                .splineTo(new Vector2d(24, -12), Math.toRadians(180))
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
                    neoArmWrapper.SetWheelSpin(1);
                    neoArmWrapper.WristDown();
                    neoArmWrapper.ClosePos();

                    neoArmWrapper.MoveExtensionMotors(-20);
                    neoArmWrapper.MoveActuatorMotor(0);

                })
                .UNSTABLE_addTemporalMarkerOffset(2, () -> {
                    neoArmWrapper.SetWheelSpin(0);
                })
                .splineTo(new Vector2d(-59.5, -10), Math.toRadians(180))
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
                    neoArmWrapper.UpdateIntakePower(0);
                    neoArmWrapper.ClosePos();
                    neoArmWrapper.WristUp();
                })
                .splineTo(new Vector2d(24, -12), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(.5, () -> {
                    neoArmWrapper.MoveExtensionMotors(1100);
                    neoArmWrapper.MoveActuatorMotor(1400);
                    neoArmWrapper.armWristServo.setPosition(.05);
                })
                .splineTo(new Vector2d(49, -36), Math.toRadians(0))
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
                .lineToLinearHeading(new Pose2d(48,-63, Math.toRadians(92)))
                .waitSeconds(3)
                .build();

        trajectory2 = drive.trajectorySequenceBuilder(new Pose2d(12, -62, Math.toRadians(-90)))
                .setReversed(true)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    neoArmWrapper.OpenPos();
                    neoArmWrapper.WristDown();
                })
                .splineTo(new Vector2d(11, -37.5),Math.toRadians(120))
                .setReversed(false)
                .UNSTABLE_addTemporalMarkerOffset(.1, () -> {
                    neoArmWrapper.ClosePos();
                })
                .UNSTABLE_addTemporalMarkerOffset(.3, () -> {
                    neoArmWrapper.WristUp();
                })
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(.5, () -> {

 //                   neoArmWrapper.setArmPositions(1000, 650);
                   neoArmWrapper.MoveExtensionMotors(852);
                   neoArmWrapper.MoveActuatorMotor(1763);
                   neoArmWrapper.armWristServo.setPosition(.1);
                })
                .splineTo(new Vector2d(50, -36), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(-.1, () -> {
                    neoArmWrapper.SetWheelSpin(-1);
                })
                .UNSTABLE_addTemporalMarkerOffset(1.5, () -> {
                    neoArmWrapper.SetWheelSpin(0);
                })
                .waitSeconds(1.5)
                .setReversed(true)
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    neoArmWrapper.armWristServo.setPosition(NeoArmWrapper.arm_wrist_intake_pos);
                })
                .splineTo(new Vector2d(24, -12), Math.toRadians(180))
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
                        neoArmWrapper.SetWheelSpin(1);
                        neoArmWrapper.WristDown();
                        neoArmWrapper.ClosePos();

                        neoArmWrapper.MoveExtensionMotors(-20);
                        neoArmWrapper.MoveActuatorMotor(0);

                        })
                .UNSTABLE_addTemporalMarkerOffset(2, () -> {
                        neoArmWrapper.SetWheelSpin(0);
                        })
                .splineTo(new Vector2d(-59.5, -10), Math.toRadians(180))
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
                    neoArmWrapper.UpdateIntakePower(0);
                    neoArmWrapper.ClosePos();
                    neoArmWrapper.WristUp();
                })
                .splineTo(new Vector2d(24, -12), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(.5, () -> {
                    neoArmWrapper.MoveExtensionMotors(1100);
                    neoArmWrapper.MoveActuatorMotor(1400);
                    neoArmWrapper.armWristServo.setPosition(.05);
                })
                .splineTo(new Vector2d(49, -42), Math.toRadians(0))
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
                .lineToLinearHeading(new Pose2d(48,-63, Math.toRadians(92)))
                .waitSeconds(3)
                .build();


        trajectory3 = drive.trajectorySequenceBuilder(new Pose2d(12, -62, Math.toRadians(-90)))
                .setReversed(true)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    neoArmWrapper.OpenPos();
                    neoArmWrapper.WristDown();
                })
                .splineTo(new Vector2d(16n



                        , -44),Math.toRadians(45))
                .setReversed(false)
                .UNSTABLE_addTemporalMarkerOffset(.1, () -> {
                    neoArmWrapper.ClosePos();
                })
                .UNSTABLE_addTemporalMarkerOffset(.3, () -> {
                    neoArmWrapper.WristUp();
                })
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(.5, () -> {

                    //                   neoArmWrapper.setArmPositions(1000, 650);
                    neoArmWrapper.MoveExtensionMotors(852);
                    neoArmWrapper.MoveActuatorMotor(1763);
                    neoArmWrapper.armWristServo.setPosition(.1);
                })
                .splineTo(new Vector2d(50, -42), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(-.1, () -> {
                    neoArmWrapper.SetWheelSpin(-1);
                })
                .UNSTABLE_addTemporalMarkerOffset(1.5, () -> {
                    neoArmWrapper.SetWheelSpin(0);
                })
                .waitSeconds(1.5)
                .setReversed(true)
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    neoArmWrapper.armWristServo.setPosition(NeoArmWrapper.arm_wrist_intake_pos);
                })
                .splineTo(new Vector2d(24, -12), Math.toRadians(180))
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
                    neoArmWrapper.SetWheelSpin(1);
                    neoArmWrapper.WristDown();
                    neoArmWrapper.ClosePos();

                    neoArmWrapper.MoveExtensionMotors(-20);
                    neoArmWrapper.MoveActuatorMotor(0);

                })
                .UNSTABLE_addTemporalMarkerOffset(2, () -> {
                    neoArmWrapper.SetWheelSpin(0);
                })
                .splineTo(new Vector2d(-59.5, -10), Math.toRadians(180))
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
                    neoArmWrapper.UpdateIntakePower(0);
                    neoArmWrapper.ClosePos();
                    neoArmWrapper.WristUp();
                })
                .splineTo(new Vector2d(24, -12), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(.5, () -> {
                    neoArmWrapper.MoveExtensionMotors(1100);
                    neoArmWrapper.MoveActuatorMotor(1400);
                    neoArmWrapper.armWristServo.setPosition(.05);
                })
                .splineTo(new Vector2d(49, -36
                ), Math.toRadians(0))
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
                .lineToLinearHeading(new Pose2d(48,-63, Math.toRadians(92)))
                .waitSeconds(3)
                .build();


        redPropWrapper.initTfod();
        while (!isStarted()){
            barcodeInt = redPropWrapper.updateTfod();
        }
        waitForStart();

        neoArmWrapper.ActivateLoop();


        if (barcodeInt == 1) {
            drive.followTrajectorySequence(trajectory1);
        } else if (barcodeInt == 2) {
            //drive.followTrajectorySequenceAsync(trajectory2);
            drive.followTrajectorySequence(trajectory2);
        } else {
            drive.followTrajectorySequence(trajectory3);
        }
        PoseStorage.currentPose = drive.getPoseEstimate();
        neoArmWrapper.DeactivateLoop();
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






    }

