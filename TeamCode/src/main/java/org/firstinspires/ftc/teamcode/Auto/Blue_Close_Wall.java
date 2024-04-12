package org.firstinspires.ftc.teamcode.Auto;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.OpModes.RedOrBlue;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.wrappers.NeoArmWrapper;

@Config
@Autonomous
public class Blue_Close_Wall extends BaseAutoOp {

    public static double T1_purplePixelLocation_x = 33;
    public static double T1_purplePixelLocation_y =40;
    public static double T1_purplePixelLocation_heading = 0;


    public static double T1_dropYellow_x = 46.5;
    public static double T1_dropYellow_y =43;
    public static double T1_dropYellow_heading = 0;


    public static int T1_dropYellow_act = 1763;
    public static int T1_dropYellow_ext = 852;
    public static double T1_dropYellow_wrist = .1;

    public static double T1_pickUpWhite_x = 24;
    public static double T1_pickUpWhite_y =60;
    public static double T1_pickUpWhite_heading = 180;



    public static double T1_pickUpWhite2_x = -56;
    public static double T1_pickUpWhite2_y =60;
    public static double T1_pickUpWhite2_heading = 180;


    public static double T1_dropWhite_x = 50;
    public static double T1_dropWhite_y =16;
    public static double T1_dropWhite_heading = 45;

    public static double T1_final_heading = -91;



    public static double T2_purplePixelLocation_x = 26;
    public static double T2_purplePixelLocation_y =37;
    public static double T2_purplePixelLocation_heading = 30;

    public static double T2_dropYellow_x = 46.5;
    public static double T2_dropYellow_y =36.5;
    public static double T2_dropYellow_heading = 0;

    public static int T2_dropYellow_act = 1763;
    public static int T2_dropYellow_ext = 852;
    public static double T2_dropYellow_wrist = .1;

    public static double T2_pickUpWhite_x = -56;
    public static double T2_pickUpWhite_y =11;
    public static double T2_pickUpWhite_heading = 180;

    public static double T2_dropWhite_x = 50;
    public static double T2_dropWhite_y =16;
    public static double T2_dropWhite_heading = 45;

    public static double T2_final_x = 48;
    public static double T2_final_y =60;
    public static double T2_final_heading = -90;




    public static double T3_purplePixelLocation_x = 13;
    public static double T3_purplePixelLocation_y =41;
    public static double T3_purplePixelLocation_heading = 0;



    public static double T3_dropYellow_x = 46.5;
    public static double T3_dropYellow_y =28;
    public static double T3_dropYellow_heading = 0;


    public static int T3_dropYellow_act = 1763;
    public static int T3_dropYellow_ext = 852;
    public static double T3_dropYellow_wrist = .1;

    public static double T3_pickUpWhite1_x = 24;
    public static double T3_pickUpWhite1_y =12;
    public static double T3_pickUpWhite1_heading = 180;



    public static double T3_pickUpWhite2_x = -56;
    public static double T3_pickUpWhite2_y =11;

    public static double T3_pickUpWhite2_heading = 180;



    public static double T3_dropWhite_x = 42.8;
    public static double T3_dropWhite_y =36;
    public static double T3_dropWhite_heading = 0;



    public static double T3_final_heading = -95;

    RedOrBlue redOrBlue;



    @Override
    public void runOpMode() throws InterruptedException {

        initialize(new Pose2d(15,62, Math.toRadians(90)), 1);
        redOrBlue.setBlue();



        TrajectorySequence trajectory1;
        TrajectorySequence trajectory2;
        TrajectorySequence trajectory3;



        TrajectorySequenceBuilder trajectory2SequenceBuilder1 = setInitialPose(new Pose2d(12, 62, Math.toRadians(90)));
        getArmReady(trajectory2SequenceBuilder1);
        dropPurplePixelLine(trajectory2SequenceBuilder1,
                new Vector2d(T1_purplePixelLocation_x, T1_purplePixelLocation_y), Math.toRadians(T1_purplePixelLocation_heading)).build();
        //getArmReadyForYellowPixelDrop(trajectory2SequenceBuilder1, T1_dropYellow_ext, T1_dropYellow_act, T1_dropYellow_wrist);
        performYellowPixelDrop(trajectory2SequenceBuilder1,
                new Vector2d(T1_dropYellow_x, T1_dropYellow_y), Math.toRadians(T1_dropYellow_heading));
        trajectory1 = pickUpWhitePixelsFar(trajectory2SequenceBuilder1,
                new Vector2d(T1_pickUpWhite_x, T1_pickUpWhite_y), Math.toRadians(T1_pickUpWhite_heading),
                new Vector2d(T1_pickUpWhite2_x, T1_pickUpWhite2_y), Math.toRadians(T1_pickUpWhite2_heading)).build();

//        trajectory1 = dropOffWhitePixels(trajectory2SequenceBuilder1,
//                new Vector2d(T1_pickUpWhite_x, T1_pickUpWhite_y), Math.toRadians(0),
//                new Vector2d(T1_dropWhite_x, T1_dropWhite_y), Math.toRadians(T1_dropWhite_heading), 500, 1200
//        ).build();

       // trajectory1 = park(trajectory2SequenceBuilder1, new Pose2d(T2_final_x,T2_final_y, Math.toRadians(T1_final_heading))).build();





        TrajectorySequenceBuilder trajectory2SequenceBuilder = setInitialPose(new Pose2d(12, 62, Math.toRadians(90)));
        getArmReady(trajectory2SequenceBuilder);
        dropPurplePixelLine(trajectory2SequenceBuilder, new Vector2d(T2_purplePixelLocation_x, T2_purplePixelLocation_y), Math.toRadians(T2_purplePixelLocation_heading)).build();
        //getArmReadyForYellowPixelDrop(trajectory2SequenceBuilder, T2_dropYellow_ext, T2_dropYellow_act, T2_dropYellow_wrist);

       performYellowPixelDrop(trajectory2SequenceBuilder,
                new Vector2d(T2_dropYellow_x, T2_dropYellow_y), Math.toRadians(T2_dropYellow_heading),500,900);
        pickUpWhitePixels(trajectory2SequenceBuilder,
                new Vector2d(24, 12), Math.toRadians(180),
                new Vector2d(T2_pickUpWhite_x, T2_pickUpWhite_y), Math.toRadians(T2_pickUpWhite_heading));

        trajectory2 = dropOffWhitePixels(trajectory2SequenceBuilder,
                new Vector2d(24, 12), Math.toRadians(0),
                new Vector2d(T2_dropWhite_x, T2_dropWhite_y), Math.toRadians(T2_dropWhite_heading), 500, 1200, NeoArmWrapper.EPixelHolderLocation.SINGLE).build();

//        pickUpWhitePixels(trajectory2SequenceBuilder,
//                new Vector2d(24, 12), Math.toRadians(180),
//                new Vector2d(T2_pickUpWhite_x, T2_pickUpWhite_y), Math.toRadians(T2_pickUpWhite_heading));
//
//        dropOffWhitePixels(trajectory2SequenceBuilder,
//                new Vector2d(24, 12), Math.toRadians(0),
//                new Vector2d(T2_dropWhite_x, T2_dropWhite_y), Math.toRadians(T2_dropWhite_heading), 500, 1200);


       // trajectory2 = park(trajectory2SequenceBuilder, new Pose2d(T2_final_x,T2_final_y, Math.toRadians(T2_final_heading))).build();




        TrajectorySequenceBuilder trajectory2SequenceBuilder3 = setInitialPose(new Pose2d(12, 62, Math.toRadians(90)));
        getArmReady(trajectory2SequenceBuilder3);
        dropPurplePixelLine(trajectory2SequenceBuilder3,
                new Vector2d(T3_purplePixelLocation_x, T3_purplePixelLocation_y), Math.toRadians(T3_purplePixelLocation_heading)).build();
        //getArmReadyForYellowPixelDrop(trajectory2SequenceBuilder3, T3_dropYellow_ext, T3_dropYellow_act, T3_dropYellow_wrist);
        performYellowPixelDrop(trajectory2SequenceBuilder3,
                new Vector2d(T3_dropYellow_x, T3_dropYellow_y), Math.toRadians(T3_dropYellow_heading));
        pickUpWhitePixels(trajectory2SequenceBuilder3,
                new Vector2d(T3_pickUpWhite1_x, T3_pickUpWhite1_y), Math.toRadians(T3_pickUpWhite1_heading),
                new Vector2d(T3_pickUpWhite2_x, T3_pickUpWhite2_y), Math.toRadians(T3_pickUpWhite2_heading));

        trajectory3 = dropOffWhitePixels(trajectory2SequenceBuilder3,
                new Vector2d(T3_pickUpWhite1_x, T3_pickUpWhite1_y), Math.toRadians(0),
                new Vector2d(T3_dropWhite_x, T3_dropWhite_y), Math.toRadians(T3_dropWhite_heading), 600, 1200, NeoArmWrapper.EPixelHolderLocation.SINGLE ).build();

        //trajectory3 = park(trajectory2SequenceBuilder3, new Pose2d(T2_final_x,T2_final_y, Math.toRadians(T3_final_heading))).build();



        neoArmWrapper.setIntakeNew();
        blueClosePropWrapper.initTfod();
        while (!isStarted()){ //TODO: MAKE SURE TO USE updateTfod(), NOT detect()
            barcodeInt = blueClosePropWrapper.updateTfod();
            neoArmWrapper.UpdateExtensionPlusInput(null, 300, 300, null, imu);
        }
        barcodeInt = 1; //TODO: MAKE SURE  TO GET RID
        waitForStart();
        run();
        if (barcodeInt == 1) {
            drive.followTrajectorySequence(trajectory1);
        } else if (barcodeInt == 2) {
            drive.followTrajectorySequence(trajectory2);
        } else {
            drive.followTrajectorySequence(trajectory3);
        }

        neoArmWrapper.setIntakeNew();
        while (opModeIsActive()) {
            neoArmWrapper.UpdateExtensionPlusInput(null, 300, 300, null, imu);
        }

    }


}
