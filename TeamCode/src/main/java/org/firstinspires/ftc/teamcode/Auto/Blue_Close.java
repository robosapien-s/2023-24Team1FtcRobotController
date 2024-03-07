package org.firstinspires.ftc.teamcode.Auto;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.wrappers.BlueClosePropWrapper;
import org.firstinspires.ftc.teamcode.wrappers.BlueFarPropWrapper;

@Autonomous
public class Blue_Close extends BaseAutoOp {

    public static double T1_purplePixelLocation_x = 16;
    public static double T1_purplePixelLocation_y =44;
    public static double T1_purplePixelLocation_heading = -135;


    public static double T1_dropYellow_x = 50;
    public static double T1_dropYellow_y =42;
    public static double T1_dropYellow_heading = 0;


    public static int T1_dropYellow_act = 1763;
    public static int T1_dropYellow_ext = 852;
    public static double T1_dropYellow_wrist = .1;

    public static double T1_pickUpWhite_x = 24;
    public static double T1_pickUpWhite_y =12;
    public static double T1_pickUpWhite_heading = 180;



    public static double T1_pickUpWhite2_x = -59.5;
    public static double T1_pickUpWhite2_y =12;
    public static double T1_pickUpWhite2_heading = 180;



    public static double T1_dropWhite_x = 49;
    public static double T1_dropWhite_y =36;
    public static double T1_dropWhite_heading = 0;



    public static double T2_purplePixelLocation_x = 12;
    public static double T2_purplePixelLocation_y =35.5;
    public static double T2_purplePixelLocation_heading = -65;

    public static double T2_dropYellow_x = 50;
    public static double T2_dropYellow_y =36;
    public static double T2_dropYellow_heading = 0;

    public static int T2_dropYellow_act = 1763;
    public static int T2_dropYellow_ext = 852;
    public static double T2_dropYellow_wrist = .1;

    public static double T2_pickUpWhite_x = -59.5;
    public static double T2_pickUpWhite_y =11;
    public static double T2_pickUpWhite_heading = 180;

    public static double T2_dropWhite_x = 49;
    public static double T2_dropWhite_y =42;
    public static double T2_dropWhite_heading = 0;

    public static double T2_final_x = 48;
    public static double T2_final_y =63;
    public static double T2_final_heading = 92;




    public static double T3_purplePixelLocation_x = 6;
    public static double T3_purplePixelLocation_y =42;
    public static double T3_purplePixelLocation_heading = -45;


    public static double T3_dropYellow_x = 50;
    public static double T3_dropYellow_y =42;
    public static double T3_dropYellow_heading = 0;


    public static int T3_dropYellow_act = 1763;
    public static int T3_dropYellow_ext = 852;
    public static double T3_dropYellow_wrist = .1;

    public static double T3_pickUpWhite1_x = 24;
    public static double T3_pickUpWhite1_y =12;
    public static double T3_pickUpWhite1_heading = 180;



    public static double T3_pickUpWhite2_x = -59.5;
    public static double T3_pickUpWhite2_y =12;
    public static double T3_pickUpWhite2_heading = 180;



    public static double T3_dropWhite_x = 48.5;
    public static double T3_dropWhite_y =36;
    public static double T3_dropWhite_heading = 0;



    @Override
    public void runOpMode() throws InterruptedException {

        initialize(new Pose2d(15,62, Math.toRadians(90)), 1);



        TrajectorySequence trajectory1;
        TrajectorySequence trajectory2;
        TrajectorySequence trajectory3;



        TrajectorySequenceBuilder trajectory2SequenceBuilder1 = setInitialPose(new Pose2d(12, 62, Math.toRadians(90)));
        getArmReady(trajectory2SequenceBuilder1);
        dropPurplePixel(trajectory2SequenceBuilder1,
                new Vector2d(T1_purplePixelLocation_x, T1_purplePixelLocation_y), Math.toRadians(T1_purplePixelLocation_heading)).build();
        getArmReadyForYellowPixelDrop(trajectory2SequenceBuilder1, T1_dropYellow_ext, T1_dropYellow_act, T1_dropYellow_wrist);
        performYellowPixelDrop(trajectory2SequenceBuilder1,
                new Vector2d(T1_dropYellow_x, T1_dropYellow_y), Math.toRadians(T1_dropYellow_heading));
        pickUpWhitePixels(trajectory2SequenceBuilder1,
                new Vector2d(24, 12), Math.toRadians(180),
                new Vector2d(T1_pickUpWhite_x, T1_pickUpWhite_y), Math.toRadians(T1_pickUpWhite_heading));

        dropOffWhitePixels(trajectory2SequenceBuilder1,
                new Vector2d(24, 12), Math.toRadians(0),
                new Vector2d(T1_dropWhite_x, T1_dropWhite_y), Math.toRadians(T1_dropWhite_heading)
        );

        trajectory1 = park(trajectory2SequenceBuilder1, new Pose2d(T2_final_x,T2_final_y, Math.toRadians(T2_final_heading))).build();


//        trajectory1 = drive.trajectorySequenceBuilder(new Pose2d(12, -62, Math.toRadians(-90)))
//                .setReversed(true)
//                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
//                    neoArmWrapper.OpenPos();
//                    neoArmWrapper.WristDown();
//
//                })
//                .splineTo(new Vector2d(6, -42),Math.toRadians(135))
//                .setReversed(false)
//                .UNSTABLE_addTemporalMarkerOffset(.1, () -> {
//                    neoArmWrapper.ClosePos();
//                })
//                .UNSTABLE_addTemporalMarkerOffset(.3, () -> {
//                    neoArmWrapper.WristUp();
//                })
//                .waitSeconds(1)
//                .UNSTABLE_addTemporalMarkerOffset(.5, () -> {
//
//                    //                   neoArmWrapper.setArmPositions(1000, 650);
//                    neoArmWrapper.MoveExtensionMotors(852);
//                    neoArmWrapper.MoveActuatorMotor(1763);
//                    neoArmWrapper.armWristServo.setPosition(.1);
//                })
//                .splineTo(new Vector2d(50, -30), Math.toRadians(0))
//                .UNSTABLE_addTemporalMarkerOffset(-.1, () -> {
//                    neoArmWrapper.SetWheelSpin(-1);
//                })
//                .UNSTABLE_addTemporalMarkerOffset(1.5, () -> {
//                    neoArmWrapper.SetWheelSpin(0);
//                })
//                .waitSeconds(1.5)
//                .setReversed(true)
//                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
//                    neoArmWrapper.armWristServo.setPosition(NeoArmWrapper.arm_wrist_intake_pos);
//                })
//                .UNSTABLE_addTemporalMarkerOffset(0.7, () -> {
//                    neoArmWrapper.MoveExtensionMotors(-20);
//                    neoArmWrapper.MoveActuatorMotor(0);
//                })
//                .splineTo(new Vector2d(24, -12), Math.toRadians(180))
//                .waitSeconds(.5)
//                .UNSTABLE_addTemporalMarkerOffset(-0.7, () -> {
//                    neoArmWrapper.SetWheelSpin(1);
//                    neoArmWrapper.WristDown();
//                    neoArmWrapper.ClosePos();
//
//                })
//                .UNSTABLE_addTemporalMarkerOffset(2, () -> {
//                    neoArmWrapper.SetWheelSpin(-0.2);
//                })
//                .splineTo(new Vector2d(-59.5, -11), Math.toRadians(180))
//                .UNSTABLE_addTemporalMarkerOffset(.1, () -> {
//                    neoArmWrapper.UpdateIntakePower(1);
//                })
//                .UNSTABLE_addTemporalMarkerOffset(1.1, () -> {
//                    neoArmWrapper.OpenPos();
//                })
//                .UNSTABLE_addTemporalMarkerOffset(2.1, () -> {
//                    neoArmWrapper.ClosePos();
//                })
//                .UNSTABLE_addTemporalMarkerOffset(3.1, () -> {
//                    neoArmWrapper.OpenPos();
//                })
//                .UNSTABLE_addTemporalMarkerOffset(3.6, () -> {
//                    neoArmWrapper.ClosePos();
//                })
//                .waitSeconds(3.7)
//                .setReversed(false)
//                .UNSTABLE_addTemporalMarkerOffset(1.5, () -> {
//                    neoArmWrapper.SetWheelSpin(0);
//                    neoArmWrapper.UpdateIntakePower(0);
//                    neoArmWrapper.ClosePos();
//                    neoArmWrapper.WristUp();
//                })
//                .splineTo(new Vector2d(24, -12), Math.toRadians(0))
//                .UNSTABLE_addTemporalMarkerOffset(.5, () -> {
//                    neoArmWrapper.MoveExtensionMotors(1100);
//                    neoArmWrapper.MoveActuatorMotor(1400);
//                    neoArmWrapper.armWristServo.setPosition(.05);
//                })
//                .splineTo(new Vector2d(49, -36), Math.toRadians(0))
//                .UNSTABLE_addTemporalMarkerOffset(-.4, () -> {
//                    neoArmWrapper.SetWheelSpin(-1);
//                })
//                .UNSTABLE_addTemporalMarkerOffset(2, () -> {
//                    neoArmWrapper.SetWheelSpin(0);
//                })
//                .waitSeconds(2.5)
//                .UNSTABLE_addTemporalMarkerOffset(.5, () -> {
//                    neoArmWrapper.armWristServo.setPosition(NeoArmWrapper.arm_wrist_intake_pos);
//                    neoArmWrapper.MoveActuatorMotor(0);
//                })
//                .UNSTABLE_addTemporalMarkerOffset(2, () -> {
//                    neoArmWrapper.MoveExtensionMotors(-20);
//                })
//                .lineToLinearHeading(new Pose2d(48,-63, Math.toRadians(92)))
//                .waitSeconds(3)
//                .build();







        TrajectorySequenceBuilder trajectory2SequenceBuilder = setInitialPose(new Pose2d(12, 62, Math.toRadians(90)));
        getArmReady(trajectory2SequenceBuilder);
        dropPurplePixel(trajectory2SequenceBuilder,
                new Vector2d(T2_purplePixelLocation_x, T2_purplePixelLocation_y), Math.toRadians(T2_purplePixelLocation_heading)).build();
        getArmReadyForYellowPixelDrop(trajectory2SequenceBuilder, T2_dropYellow_ext, T2_dropYellow_act, T2_dropYellow_wrist);
        performYellowPixelDrop(trajectory2SequenceBuilder,
                new Vector2d(T2_dropYellow_x, T2_dropYellow_y), Math.toRadians(T2_dropYellow_heading));
        pickUpWhitePixels(trajectory2SequenceBuilder,
                new Vector2d(24, 12), Math.toRadians(180),
                new Vector2d(T2_pickUpWhite_x, T2_pickUpWhite_y), Math.toRadians(T2_pickUpWhite_heading));
        dropOffWhitePixels(trajectory2SequenceBuilder,
                new Vector2d(24, 12), Math.toRadians(0),
                new Vector2d(T2_dropWhite_x, T2_dropWhite_y), Math.toRadians(T2_dropWhite_heading));


        trajectory2 = park(trajectory2SequenceBuilder, new Pose2d(T2_final_x,T2_final_y, Math.toRadians(T2_final_heading))).build();


        TrajectorySequenceBuilder trajectory2SequenceBuilder3 = setInitialPose(new Pose2d(12, 62, Math.toRadians(90)));
        getArmReady(trajectory2SequenceBuilder3);
        dropPurplePixel(trajectory2SequenceBuilder3,
                new Vector2d(T3_purplePixelLocation_x, T3_purplePixelLocation_y), Math.toRadians(T3_purplePixelLocation_heading)).build();
        getArmReadyForYellowPixelDrop(trajectory2SequenceBuilder3, T3_dropYellow_ext, T3_dropYellow_act, T3_dropYellow_wrist);
        performYellowPixelDrop(trajectory2SequenceBuilder3,
                new Vector2d(T3_dropYellow_x, T3_dropYellow_y), Math.toRadians(T3_dropYellow_heading));
        pickUpWhitePixels(trajectory2SequenceBuilder3,
                new Vector2d(T3_pickUpWhite1_x, T3_pickUpWhite1_y), Math.toRadians(T3_pickUpWhite1_heading),
                new Vector2d(T3_pickUpWhite2_x, T3_pickUpWhite2_y), Math.toRadians(T3_pickUpWhite2_heading));

        dropOffWhitePixels(trajectory2SequenceBuilder3,
                new Vector2d(24, 12), Math.toRadians(0),
                new Vector2d(T3_dropWhite_x, T3_dropWhite_y), Math.toRadians(T3_dropWhite_heading) );

        trajectory3 = park(trajectory2SequenceBuilder3, new Pose2d(T2_final_x,T2_final_y, Math.toRadians(T2_final_heading))).build();



        blueClosePropWrapper.initTfod();
        while (!isStarted()){ //TODO: MAKE SURE TO USE updateTfod(), NOT detect()
            barcodeInt = blueClosePropWrapper.updateTfod();
        }
        waitForStart();

        if (barcodeInt == 1) {
            drive.followTrajectorySequence(trajectory1);
        } else if (barcodeInt == 2) {
            drive.followTrajectorySequence(trajectory2);
        } else {
            drive.followTrajectorySequence(trajectory3);
        }

    }


}
