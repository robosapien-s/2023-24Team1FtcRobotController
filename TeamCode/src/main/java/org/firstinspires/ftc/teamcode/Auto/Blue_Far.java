package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.OpModes.RedOrBlue;
import org.firstinspires.ftc.teamcode.drive.PoseStorage;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.wrappers.NeoArmWrapper;

@Config
@Autonomous
public class Blue_Far extends BaseAutoOp {





    public static double start_x = -43;
    public static double start_y =62;
    public static double start_heading = 90;


    //Purple pixel location
    public static double T3_purplePixelLocation_x = -38.5;
    public static double T3_purplePixelLocation_y =33.5;
    public static double T3_purplePixelLocation_heading = 0;


    public static double T2_purplePixelLocation_x = -50;
    public static double T2_purplePixelLocation_y =37;
    public static double T2_purplePixelLocation_heading = 90;


    public static double T1_purplePixelLocation_x = -41.5;
    public static double T1_purplePixelLocation_y =29;
    public static double T1_purplePixelLocation_heading = 180;



    //lineup for single white
    public static double T3_lineUpSingleWhite_x = -36;
    public static double T3_lineUpSingleWhite_y =12;
    public static double T3_lineUpSingleWhite_heading = 0;


    public static double T2_lineUpSingleWhite_x = -56;
    public static double T2_lineUpSingleWhite_y =24;
    public static double T2_lineUpSingleWhite_heading = 0;


    public static double T1_lineUpSingleWhite_x = -40;
    public static double T1_lineUpSingleWhite_y =12.5;
    public static double T1_lineUpSingleWhite_heading = 0;



    //Pickup Pixels
    public static double T3_pickUpWhite2_x = -61;
    public static double T3_pickUpWhite2_y =11;
    public static double T3_pickUpWhite2_heading = 0;  //Note trajectory is reversed


    public static double T2_pickUpWhite2_x = -63;
    public static double T2_pickUpWhite2_y = 24;
    public static double T2_pickUpWhite2_heading = 180;  //Note trajectory is reversed

    public static double T1_pickUpWhite2_x = -62.5;
    public static double T1_pickUpWhite2_y =11.4;
    public static double T1_pickUpWhite2_heading = 0;  //Note trajectory is reversed


    //Drop off
    public static double T3_dropWhite_x = 43.5;

    public static double T3_dropWhite_y =36;
    public static double T3_second_dropWhite_y =42;
    public static double T3_dropWhite_heading = 0;



    public static double T2_dropWhite_x = 41;

    public static double T2_dropWhite_y =34.3;
    public static double T2_second_dropWhite_y =25;

    public static double T2_dropWhite_heading = 0;

    public static double T1_dropWhite_x =  43.5;

    public static double T1_dropWhite_y =36;


    public static double T1_second_dropWhite_y =32;
    public static double T1_dropWhite_heading = 0;




    public static double final_x = 42;
    public static double final_close_y =63;
    public static double final_far_y =12;
    public static double final_heading = -87;



    public TrajectorySequence buildTragectory1() {

        TrajectorySequenceBuilder trajectory2SequenceBuilder1 = setInitialPose((new Pose2d(start_x, start_y, Math.toRadians(start_heading))));
        getArmReady(trajectory2SequenceBuilder1);
        dropPurplePixelLine(trajectory2SequenceBuilder1,
                new Vector2d(Blue_Far.T1_purplePixelLocation_x, Blue_Far.T1_purplePixelLocation_y), Math.toRadians(Blue_Far.T1_purplePixelLocation_heading), false);
        lineUpForSinglePixelFarBackBoard(trajectory2SequenceBuilder1,
                new Vector2d(Blue_Far.T1_lineUpSingleWhite_x, Blue_Far.T1_lineUpSingleWhite_y), Math.toRadians(Blue_Far.T1_lineUpSingleWhite_heading));
        pickUpOneFarWhitePixels(trajectory2SequenceBuilder1, new Vector2d(T1_pickUpWhite2_x, T1_pickUpWhite2_y),  Math.toRadians(T1_pickUpWhite2_heading-4));
        //TODO flip pixel holder
        return dropOffWhitePixels(trajectory2SequenceBuilder1,
                new Vector2d(24, 12), Math.toRadians(0),
                new Vector2d(T1_dropWhite_x, T1_dropWhite_y), Math.toRadians(T1_dropWhite_heading), 750, 1200, NeoArmWrapper.EPixelHolderLocation.DOUBLE,2
        ).build();

//        return park(trajectory2SequenceBuilder1,
//                new Pose2d(final_x, final_far_y, Math.toRadians(final_heading))).build();

    }

    public TrajectorySequence buildTragectory2() {

        TrajectorySequenceBuilder trajectory2SequenceBuilder1 = setInitialPose((new Pose2d(start_x, start_y, Math.toRadians(start_heading))));
        getArmReady(trajectory2SequenceBuilder1);
        dropPurplePixelLine(trajectory2SequenceBuilder1,
                new Vector2d(Blue_Far.T2_purplePixelLocation_x, Blue_Far.T2_purplePixelLocation_y), Math.toRadians(Blue_Far.T2_purplePixelLocation_heading), false);
        trajectory2SequenceBuilder1.strafeLeft(12);
        //trajectory2SequenceBuilder1.strafeLeft(8);
        lineUpForSinglePixelFarBackBoard(trajectory2SequenceBuilder1,
                new Vector2d(Blue_Far.T2_lineUpSingleWhite_x, Blue_Far.T1_lineUpSingleWhite_y), Math.toRadians(Blue_Far.T1_lineUpSingleWhite_heading));
        pickUpOneFarWhitePixels(trajectory2SequenceBuilder1, new Vector2d(T1_pickUpWhite2_x, T1_pickUpWhite2_y),  Math.toRadians(T1_pickUpWhite2_heading-8));
        return dropOffWhitePixels(trajectory2SequenceBuilder1,
                new Vector2d(24, 12), Math.toRadians(0),
                new Vector2d(T2_dropWhite_x, T2_dropWhite_y), Math.toRadians(T1_dropWhite_heading), 750, 1200, NeoArmWrapper.EPixelHolderLocation.SINGLE_UPSIDE_DOWN,2
        ).build();
//        return park(trajectory2SequenceBuilder1,
//                new Pose2d(final_x, final_far_y, Math.toRadians(final_heading))).build();

    }

    public TrajectorySequence buildTragectory3() {

        TrajectorySequenceBuilder trajectory2SequenceBuilder3 = setInitialPose((new Pose2d(start_x, start_y, Math.toRadians(start_heading))));
        getArmReady(trajectory2SequenceBuilder3);
        dropPurplePixelLine(trajectory2SequenceBuilder3,
                new Vector2d(Blue_Far.T3_purplePixelLocation_x, Blue_Far.T3_purplePixelLocation_y), Math.toRadians(Blue_Far.T3_purplePixelLocation_heading), false);
        lineUpForSinglePixelFarBackBoard(trajectory2SequenceBuilder3,
                new Vector2d(Blue_Far.T3_lineUpSingleWhite_x, Blue_Far.T3_lineUpSingleWhite_y), Math.toRadians(Blue_Far.T3_lineUpSingleWhite_heading));
        pickUpOneFarWhitePixels(trajectory2SequenceBuilder3, new Vector2d(T3_pickUpWhite2_x, T3_pickUpWhite2_y),  Math.toRadians(T3_pickUpWhite2_heading));
        return dropOffWhitePixels(trajectory2SequenceBuilder3,
                new Vector2d(24, 12), Math.toRadians(0),
                new Vector2d(T3_dropWhite_x, T3_dropWhite_y), Math.toRadians(T3_dropWhite_heading), 750, 1200, NeoArmWrapper.EPixelHolderLocation.DOUBLE,2
        ).build();

//        return park(trajectory2SequenceBuilder3,
//                new Pose2d(final_x, final_far_y, Math.toRadians(final_heading))).build();

    }


    RedOrBlue redOrBlue;

    @Override
    public void runOpMode() throws InterruptedException {

        initialize(new Pose2d(start_x,start_y, Math.toRadians(start_heading)), 1);

        redOrBlue.setBlue();

        TrajectorySequence trajectory1 = buildTragectory1();
        TrajectorySequence trajectory2 = buildTragectory2();
        TrajectorySequence trajectory3 = buildTragectory3();

        neoArmWrapper.setIntakeNew();
        blueFarPropWrapper.initTfod();
        while (!isStarted()){ //TODO: MAKE SURE TO USE updateTfod(), NOT detect()
            barcodeInt = blueFarPropWrapper.updateTfod();
            neoArmWrapper.UpdateExtensionPlusInput(null, 300, 300, null, imu);
        }

        waitForStart();
        run();

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
