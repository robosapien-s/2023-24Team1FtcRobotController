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
public class Red_Close_Wall extends BaseAutoOp {









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





    public static int T3_dropYellow_act = 1763;
    public static int T3_dropYellow_ext = 852;
    public static double T3_dropYellow_wrist = .15;

    public static double T3_pickUpWhite1_x = 24;
    public static double T3_pickUpWhite1_y =-10.5;
    public static double T3_pickUpWhite1_heading = 180;




    public static double T1_purplePixelLocation_x = 12;
    public static double T1_purplePixelLocation_y =-28;
    public static double T1_purplePixelLocation_heading = 0;

    public static double T2_purplePixelLocation_x = 22;
    public static double T2_purplePixelLocation_y =-37;
    public static double T2_purplePixelLocation_heading = -90;

    public static double T3_purplePixelLocation_x = 28;
    public static double T3_purplePixelLocation_y = -46.5;
    public static double T3_purplePixelLocation_heading = -90;


    public static int T2_dropYellow_act = 1763;
    public static int T2_dropYellow_ext = 852;
    public static double T2_dropYellow_wrist = .1;


    public static double T2_dropWhite_x = 52;
    public static double T2_dropWhite_y = -55.5;
    public static double T2_dropWhite_heading = 45;


    public static double T3_dropWhite_x = 51;
    public static double T3_dropWhite_y =-55.5;
    public static double T3_dropWhite_heading = 45;



    public static double T2_final_x = 48;
    public static double T2_final_y =-63;
    public static double T2_final_heading = 92;







    public static double T1_dropYellow_x = 47.5;
    public static double T1_dropYellow_y =-30;
    public static double T1_dropYellow_heading = 0;


    public static double T2_dropYellow_x = 47.5;
    public static double T2_dropYellow_y =-36.25;
    public static double T2_dropYellow_heading = 0;


    public static double T3_dropYellow_x = 45.5;
    public static double T3_dropYellow_y =-43.5;
    public static double T3_dropYellow_heading = 0;


    public static int T1_dropYellow_act = 1763;
    public static int T1_dropYellow_ext = 852;
    public static double T1_dropYellow_wrist = .1;

    public static double T1_pickUpWhite1_x = 24;
    public static double T1_pickUpWhite1_y =-12;
    public static double T1_pickUpWhite_heading = 150;


    public static double T1_pickUpWhite_x = -57.5;
    public static double T1_pickUpWhite_y =-37;
    public static double T1_pickUpWhite2_heading = 180;

    public static double T2_pickUpWhite_x = -57.5;
    public static double T2_pickUpWhite_y =-37;
    public static double T2_pickUpWhite_heading = 150;


    public static double T3_pickUpWhite2_x = -57.5;
    public static double T3_pickUpWhite2_y =-36.5;
    public static double T3_pickUpWhite2_heading = 150;




    public static double T1_dropWhite_x = 52;
    public static double T1_dropWhite_y =-56.5;
    public static double T1_dropWhite_heading = 45;


    RedOrBlue redOrBlue;

    @Override
    public void runOpMode() throws InterruptedException {

        initialize(new Pose2d(15,-62, Math.toRadians(-90)), 1);

        redOrBlue.setRedAuto();
        redOrBlue.setRed();

        TrajectorySequence trajectory1;
        TrajectorySequence trajectory2;
        TrajectorySequence trajectory3;

        TrajectorySequenceBuilder trajectory2SequenceBuilder1 = setInitialPose(new Pose2d(12, -62, Math.toRadians(-90)));
        getArmReady(trajectory2SequenceBuilder1);
        dropPurplePixelLine(trajectory2SequenceBuilder1,
                new Vector2d(T1_purplePixelLocation_x, T1_purplePixelLocation_y), Math.toRadians(T1_purplePixelLocation_heading)).build();
        performYellowPixelDrop(trajectory2SequenceBuilder1,
                new Vector2d(T1_dropYellow_x, T1_dropYellow_y), Math.toRadians(T1_dropYellow_heading), 900, 500);
        pickUpWhitePixelsWallRed(trajectory2SequenceBuilder1,
                new Vector2d(24, -56.5), Math.toRadians(180),
                new Vector2d(-36, -56.5), Math.toRadians(180),
                new Vector2d(T1_pickUpWhite_x, T1_pickUpWhite_y), Math.toRadians(T1_pickUpWhite_heading),
                1.5
        );

        trajectory1 = dropOffWhitePixelsWall(trajectory2SequenceBuilder1,
                new Vector2d(-36, -56.5), Math.toRadians(0),
                new Vector2d(24, -56.5), Math.toRadians(0),
                new Vector2d(T1_dropWhite_x, T1_dropWhite_y), Math.toRadians(T1_dropWhite_heading)
                , 1200, 1200, NeoArmWrapper.EPixelHolderLocation.SINGLE,0
        ).build();
        //trajectory1 = park(trajectory2SequenceBuilder1, new Pose2d(T2_final_x,T2_final_y, Math.toRadians(T2_final_heading))).build();


        TrajectorySequenceBuilder trajectory2SequenceBuilder2 = setInitialPose(new Pose2d(12, -62, Math.toRadians(-90)));
        getArmReady(trajectory2SequenceBuilder2);
        dropPurplePixelLine(trajectory2SequenceBuilder2,
                new Vector2d(T2_purplePixelLocation_x, T2_purplePixelLocation_y), Math.toRadians(T2_purplePixelLocation_heading)).build();
        performYellowPixelDrop(trajectory2SequenceBuilder2,
                new Vector2d(T2_dropYellow_x, T2_dropYellow_y), Math.toRadians(T2_dropYellow_heading), 900, 500);
        pickUpWhitePixelsWallRed(trajectory2SequenceBuilder2,
                new Vector2d(24, -56.5), Math.toRadians(180),
                new Vector2d(-36, -56.5), Math.toRadians(180),
                new Vector2d(T2_pickUpWhite_x, T2_pickUpWhite_y), Math.toRadians(T2_pickUpWhite_heading),
                1.5
        );

        trajectory2 = dropOffWhitePixelsWall(trajectory2SequenceBuilder2,
                new Vector2d(-36, -56.5), Math.toRadians(0),
                new Vector2d(24, -56.5), Math.toRadians(0),
                new Vector2d(T2_dropWhite_x, T2_dropWhite_y), Math.toRadians(T2_dropWhite_heading)
                , 1200, 1200, NeoArmWrapper.EPixelHolderLocation.SINGLE,0
        ).build();
        //trajectory2 = park(trajectory2SequenceBuilder, new Pose2d(T2_final_x,T2_final_y, Math.toRadians(T2_final_heading))).build();


        TrajectorySequenceBuilder trajectory2SequenceBuilder3 = setInitialPose(new Pose2d(12, -62, Math.toRadians(-90)));
        getArmReady(trajectory2SequenceBuilder3);
        dropPurplePixelLine(trajectory2SequenceBuilder3,
                new Vector2d(T3_purplePixelLocation_x, T3_purplePixelLocation_y), Math.toRadians(T3_purplePixelLocation_heading)).build();
        performYellowPixelDrop(trajectory2SequenceBuilder3,
                new Vector2d(T3_dropYellow_x, T3_dropYellow_y), Math.toRadians(T3_dropYellow_heading),900,500);
        pickUpWhitePixelsWallRed(trajectory2SequenceBuilder3,
                new Vector2d(24, -56.5), Math.toRadians(180),
                new Vector2d(-36, -56.5), Math.toRadians(180),
                new Vector2d(T3_pickUpWhite2_x, T3_pickUpWhite2_y), Math.toRadians(T3_pickUpWhite2_heading),
                1.5
        );

        trajectory3 = dropOffWhitePixelsWall(trajectory2SequenceBuilder3,
                new Vector2d(-36, -56.5), Math.toRadians(0),
                new Vector2d(24, -56.5), Math.toRadians(0),
                new Vector2d(T3_dropWhite_x, T3_dropWhite_y), Math.toRadians(T3_dropWhite_heading)
                , 1200, 1250, NeoArmWrapper.EPixelHolderLocation.SINGLE,0
        ).build();
        //trajectory3 = park(trajectory2SequenceBuilder3, new Pose2d(T2_final_x,T2_final_y, Math.toRadians(T2_final_heading))).build();


        neoArmWrapper.setIntakeNew();
        redClosePropWrapper.initTfod();
        while (!isStarted()){ //TODO: MAKE SURE TO USE updateTfod(), NOT detect()
            barcodeInt = redClosePropWrapper.updateTfod();
            neoArmWrapper.UpdateExtensionPlusInput(null, 300, 300, null, imu);
        }

        waitForStart();
        run();
        //neoArmWrapper.ActivateLoop();


        if (barcodeInt == 1) {
            drive.followTrajectorySequence(trajectory1);
        } else if (barcodeInt == 2) {
            drive.followTrajectorySequence(trajectory2);
        } else {
            drive.followTrajectorySequence(trajectory3);
        }
        PoseStorage.currentPose = drive.getPoseEstimate();

        neoArmWrapper.setIntakeNew();
        while (opModeIsActive()) {
            neoArmWrapper.UpdateExtensionPlusInput(null, 300, 300, null, imu);
        }


    }


    }
