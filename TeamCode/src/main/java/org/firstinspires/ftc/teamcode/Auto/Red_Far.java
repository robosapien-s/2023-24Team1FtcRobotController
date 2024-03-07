package org.firstinspires.ftc.teamcode.Auto;
import android.graphics.drawable.VectorDrawable;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.drive.PoseStorage;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;

@Config
@Autonomous
public class Red_Far extends BaseAutoOp {





    public static double start_x = -43;
    public static double start_y =-62;
    public static double start_heading = 270;


    //Purple pixel location
    public static double T1_purplePixelLocation_x = -39;
    public static double T1_purplePixelLocation_y =-35.5;
    public static double T1_purplePixelLocation_heading = 0;


    public static double T2_purplePixelLocation_x = -43;
    public static double T2_purplePixelLocation_y =-25.5;
    public static double T2_purplePixelLocation_heading = 270;


    public static double T3_purplePixelLocation_x = -42;
    public static double T3_purplePixelLocation_y =-35.5;
    public static double T3_purplePixelLocation_heading = 180;



    //lineup for single white
    public static double T1_lineUpSingleWhite_x = -40;
    public static double T1_lineUpSingleWhite_y =-8.5;
    public static double T1_lineUpSingleWhite_heading = 0;


    public static double T2_lineUpSingleWhite_x = -36;
    public static double T2_lineUpSingleWhite_y =-36;
    public static double T2_lineUpSingleWhite_heading = 0;


    public static double T3_lineUpSingleWhite_x = -40;
    public static double T3_lineUpSingleWhite_y =-8.5;
    public static double T3_lineUpSingleWhite_heading = 0;



    //Pickup Pixels
    public static double T1_pickUpWhite2_x = -61;
    public static double T1_pickUpWhite2_y =-8.5;
    public static double T1_pickUpWhite2_heading = 180;  //Note trajectory is reversed


    public static double T2_pickUpWhite2_x = -61;
    public static double T2_pickUpWhite2_y = -36;
    public static double T2_pickUpWhite2_heading = 180;  //Note trajectory is reversed




    //Drop off
    public static double T1_dropWhite_x = 47;

    public static double T1_dropWhite_y =-34;
    public static double T1_second_dropWhite_y =-32.25;
    public static double T1_dropWhite_heading = 0;



    public static double T2_dropWhite_x = 47;

    public static double T2_dropWhite_y =-37;
    public static double T2_second_dropWhite_y =-35.25;

    public static double T2_dropWhite_heading = 0;






    public static double final_x = 42;
    public static double final_close_y =-63;
    public static double final_far_y =-20;
    public static double final_heading = 88;


    public TrajectorySequence buildTragectory1() {

        TrajectorySequenceBuilder trajectory2SequenceBuilder1 = setInitialPose((new Pose2d(start_x, start_y, Math.toRadians(start_heading))));
        getArmReady(trajectory2SequenceBuilder1);
        dropPurplePixelFar(trajectory2SequenceBuilder1,
                new Vector2d(Red_Far.T1_purplePixelLocation_x, Red_Far.T1_purplePixelLocation_y), Math.toRadians(Red_Far.T1_purplePixelLocation_heading));

        lineUpForSinglePixel(trajectory2SequenceBuilder1,
                new Vector2d(Red_Far.T1_lineUpSingleWhite_x, Red_Far.T1_lineUpSingleWhite_y), Math.toRadians(Red_Far.T1_lineUpSingleWhite_heading));


        pickUpOneFarWhitePixels(trajectory2SequenceBuilder1, new Vector2d(T1_pickUpWhite2_x, T1_pickUpWhite2_y),  Math.toRadians(T1_pickUpWhite2_heading));


        dropOffWhitePixels(trajectory2SequenceBuilder1,
                new Vector2d(24, -12), Math.toRadians(0),
                new Vector2d(T1_dropWhite_x, T1_dropWhite_y), Math.toRadians(T1_dropWhite_heading),
                new Vector2d(T1_dropWhite_x, T1_second_dropWhite_y)
        );


        return park(trajectory2SequenceBuilder1,
                new Pose2d(final_x, final_far_y, Math.toRadians(final_heading))).build();

    }

    public TrajectorySequence buildTragectory2() {

        TrajectorySequenceBuilder trajectory2SequenceBuilder1 = setInitialPose((new Pose2d(start_x, start_y, Math.toRadians(start_heading))));
        getArmReady(trajectory2SequenceBuilder1);
        dropPurplePixelFar(trajectory2SequenceBuilder1,
                new Vector2d(Red_Far.T2_purplePixelLocation_x, Red_Far.T2_purplePixelLocation_y), Math.toRadians(Red_Far.T2_purplePixelLocation_heading));

        lineUpForSinglePixel(trajectory2SequenceBuilder1,
                new Vector2d(Red_Far.T2_lineUpSingleWhite_x, Red_Far.T2_lineUpSingleWhite_y), Math.toRadians(Red_Far.T2_lineUpSingleWhite_heading));


        pickUpOneFarWhitePixels(trajectory2SequenceBuilder1, new Vector2d(T2_pickUpWhite2_x, T2_pickUpWhite2_y),  Math.toRadians(T2_pickUpWhite2_heading));


        dropOffWhitePixels(trajectory2SequenceBuilder1,
                new Vector2d(T2_dropWhite_x-3, T2_dropWhite_y), Math.toRadians(0), //hack to reuse but just get it moving and not get empty traj error
                new Vector2d(T2_dropWhite_x, T2_dropWhite_y), Math.toRadians(T2_dropWhite_heading),
                new Vector2d(T2_dropWhite_x, T2_second_dropWhite_y)
        );


        return park(trajectory2SequenceBuilder1,
                new Pose2d(final_x, final_far_y, Math.toRadians(final_heading))).build();

    }


    public TrajectorySequence buildTragectory3() {

        TrajectorySequenceBuilder trajectory2SequenceBuilder1 = setInitialPose((new Pose2d(start_x, start_y, Math.toRadians(start_heading))));
        getArmReady(trajectory2SequenceBuilder1);
        dropPurplePixelFar(trajectory2SequenceBuilder1,
                new Vector2d(Red_Far.T3_purplePixelLocation_x, Red_Far.T3_purplePixelLocation_y), Math.toRadians(Red_Far.T3_purplePixelLocation_heading));

        lineUpForSinglePixel(trajectory2SequenceBuilder1,
                new Vector2d(Red_Far.T1_lineUpSingleWhite_x, Red_Far.T1_lineUpSingleWhite_y), Math.toRadians(Red_Far.T1_lineUpSingleWhite_heading));


        pickUpOneFarWhitePixels(trajectory2SequenceBuilder1, new Vector2d(T1_pickUpWhite2_x, T1_pickUpWhite2_y),  Math.toRadians(T1_pickUpWhite2_heading));


        dropOffWhitePixels(trajectory2SequenceBuilder1,
                new Vector2d(24, -12), Math.toRadians(0),
                new Vector2d(T1_dropWhite_x, T1_dropWhite_y), Math.toRadians(T1_dropWhite_heading),
                new Vector2d(T1_dropWhite_x, T1_second_dropWhite_y)
        );


        return park(trajectory2SequenceBuilder1,
                new Pose2d(final_x, final_far_y, Math.toRadians(final_heading))).build();

    }

    @Override
    public void runOpMode() throws InterruptedException {

        initialize(new Pose2d(start_x,start_y, Math.toRadians(start_heading)), 1);

        TrajectorySequence trajectory1 = buildTragectory1();
        TrajectorySequence trajectory2 = buildTragectory2();
        TrajectorySequence trajectory3 = buildTragectory3();

        redClosePropWrapper.initTfod();
        while (!isStarted()){
            barcodeInt = redClosePropWrapper.updateTfod();
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
