package org.firstinspires.ftc.teamcode.Auto;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.PoseStorage;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.wrappers.RedPropWrapper;

@Config
@Autonomous
public class Red_Far extends BaseAutoOp {





    public static double start_x = -42;
    public static double start_y =-62;
    public static double start_heading = 270;


    public static double T1_purplePixelLocation_x = -39;
    public static double T1_purplePixelLocation_y =-36;
    public static double T1_purplePixelLocation_heading = 0;


    public static double T1_lineUpSingleWhite_x = -40;
    public static double T1_lineUpSingleWhite_y =-12;
    public static double T1_lineUpSingleWhite_heading = 0;


    public static double T1_pickUpWhite2_x = -62.9;
    public static double T1_pickUpWhite2_y =-12;
    public static double T1_pickUpWhite2_heading = 180;



    public static double T1_dropWhite_x = 49;
    public static double T1_dropWhite_y =-36;
    public static double T1_dropWhite_heading = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        initialize(new Pose2d(start_x,start_y, Math.toRadians(start_heading)), 1);


        TrajectorySequence trajectory1;
        TrajectorySequence trajectory2;
        TrajectorySequence trajectory3;




        TrajectorySequenceBuilder trajectory2SequenceBuilder1 = setInitialPose((new Pose2d(start_x, start_y, Math.toRadians(start_heading))));
        getArmReady(trajectory2SequenceBuilder1);
        dropPurplePixelFar(trajectory2SequenceBuilder1,
                new Vector2d(Red_Far.T1_purplePixelLocation_x, Red_Far.T1_purplePixelLocation_y), Math.toRadians(Red_Far.T1_purplePixelLocation_heading));

        lineUpForSinglePixel(trajectory2SequenceBuilder1,
                new Vector2d(Red_Far.T1_lineUpSingleWhite_x, Red_Far.T1_lineUpSingleWhite_y), Math.toRadians(Red_Far.T1_lineUpSingleWhite_heading));


        pickUpOneFarWhitePixels(trajectory2SequenceBuilder1, new Vector2d(T1_pickUpWhite2_x, T1_pickUpWhite2_y),  Math.toRadians(T1_pickUpWhite2_heading));


        dropOffWhitePixels(trajectory2SequenceBuilder1,
                new Vector2d(24, -12), Math.toRadians(0),
                new Vector2d(T1_dropWhite_x, T1_dropWhite_y), Math.toRadians(T1_dropWhite_heading));


        trajectory1 = park(trajectory2SequenceBuilder1,
                new Pose2d(T1_dropWhite_x-6, T1_dropWhite_y, Math.toRadians(T1_dropWhite_heading))).build();


//        trajectory1 = pickUpWhitePixels(trajectory2SequenceBuilder1,
//                new Vector2d(24, -12), Math.toRadians(180),
//                new Vector2d(T1_pickUpWhite2_x, T1_pickUpWhite2_y), Math.toRadians(T1_pickUpWhite2_heading)).build();





//        getArmReadyForYellowPixelDrop(trajectory2SequenceBuilder1, T2_dropYellow_ext, T2_dropYellow_act, T2_dropYellow_wrist);
//        performYellowPixelDrop(trajectory2SequenceBuilder1,
//                new Vector2d(T2_dropYellow_x, T2_dropYellow_y), Math.toRadians(T2_dropYellow_heading));
//        pickUpWhitePixels(trajectory2SequenceBuilder1,
//                new Vector2d(24, -12), Math.toRadians(180),
//                new Vector2d(T2_pickUpWhite_x, T2_pickUpWhite_y), Math.toRadians(T2_pickUpWhite_heading));
//        trajectory2 = dropOffWhitePixels(trajectory2SequenceBuilder1,
//                new Vector2d(24, -12), Math.toRadians(0),
//                new Vector2d(T2_dropWhite_x, T2_dropWhite_y), Math.toRadians(T2_dropWhite_heading),
//                new Pose2d(T2_final_x,T2_final_y, Math.toRadians(T2_final_heading)) ).build();

//        trajectory1 = drive.trajectorySequenceBuilder(new Pose2d(start_x, start_y, Math.toRadians(start_heading)))
//           // .setReversed(true)
//                .lineToLinearHeading( new Pose2d(Red_Far.T1_purplePixelLocation_x, Red_Far.T1_purplePixelLocation_y, Math.toRadians(Red_Far.T1_purplePixelLocation_heading)))
//            .splineTo(new Vector2d(-40, -36), Math.toRadians(135))
//            .setReversed(false)
//            .waitSeconds(.5)
//            .strafeLeft(15)
//            .setReversed(true)
//            .splineTo(new Vector2d(-60,-12), Math.toRadians(180))
//            .setReversed(false)
//            .waitSeconds(.5)
//            .splineTo(new Vector2d(24, -12), Math.toRadians(0))
//            .splineTo(new Vector2d(51, -30), Math.toRadians(0))
//            .setReversed(true)
//            .waitSeconds(.5)
//            .splineTo(new Vector2d(24, -12), Math.toRadians(180))
//            .splineTo(new Vector2d(-60, -12), Math.toRadians(180))
//            .waitSeconds(.5)
//            .setReversed(false)
//            .splineTo(new Vector2d(24, -12), Math.toRadians(0))
//            .splineTo(new Vector2d(51, -30), Math.toRadians(0))
//            .waitSeconds(.5)
//            .lineToLinearHeading(new Pose2d(48,-12, Math.toRadians(0)))
//            .lineToLinearHeading(new Pose2d(60,-12, Math.toRadians(0)))
//            .build();

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




        redPropWrapper.initTfod();
        while (!isStarted()){
            barcodeInt = redPropWrapper.updateTfod();
        }
        waitForStart();

        neoArmWrapper.ActivateLoop();

        barcodeInt = 1;

        if (barcodeInt == 1) {
            drive.followTrajectorySequence(trajectory1);
        } else if (barcodeInt == 2) {
            drive.followTrajectorySequence(trajectory2);
        } else {
            drive.followTrajectorySequence(trajectory3);
        }
        PoseStorage.currentPose = drive.getPoseEstimate();
        neoArmWrapper.DeactivateLoop();

    }


}
