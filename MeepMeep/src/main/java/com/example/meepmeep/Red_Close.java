package com.example.meepmeep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class Red_Close {
    static int barcodeInt = 3;
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        RoadRunnerBotEntity myBot = null;

        if (barcodeInt == 2) {
            myBot = new DefaultBotBuilder(meepMeep)
                    // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                    .setConstraints(52.48291908330528, 52.48291908330528, Math.toRadians(197.5918273305535), Math.toRadians(222.7444266666667), 13.26)
                    .followTrajectorySequence(drive ->
                            drive.trajectorySequenceBuilder(new Pose2d(12, -62, Math.toRadians(-90)))
                                    .lineToLinearHeading(new Pose2d(12, -32, Math.toRadians(-90)))
                                    .waitSeconds(.5)
                                    .splineTo(new Vector2d(51, -36), Math.toRadians(0))
                                    .waitSeconds(.5)
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
                                    .build()
                    );
        } else if (barcodeInt ==1) {
            myBot = new DefaultBotBuilder(meepMeep)
                    // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                    .setConstraints(52.48291908330528, 52.48291908330528, Math.toRadians(197.5918273305535), Math.toRadians(222.7444266666667), 13.26)
                    .followTrajectorySequence(drive ->
                            drive.trajectorySequenceBuilder(new Pose2d(12, -62, Math.toRadians(-90)))
                                    .setReversed(true)
                                    .splineTo(new Vector2d(6, -36), Math.toRadians(135))
                                    .setReversed(false)
                                    .waitSeconds(.5)
                                    .splineTo(new Vector2d(51, -30), Math.toRadians(0))
                                    .waitSeconds(.5)
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
                                    .build()
                    );
        } else {
            myBot = new DefaultBotBuilder(meepMeep)
                    // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                    .setConstraints(52.48291908330528, 52.48291908330528, Math.toRadians(197.5918273305535), Math.toRadians(222.7444266666667), 13.26)
                    .followTrajectorySequence(drive ->
                            drive.trajectorySequenceBuilder(new Pose2d(12, -62, Math.toRadians(-90)))
                                    .setReversed(true)
                                    .splineTo(new Vector2d(18, -36), Math.toRadians(45))
                                    .setReversed(false)
                                    .waitSeconds(.5)
                                    .splineTo(new Vector2d(51, -42), Math.toRadians(0))
                                    .waitSeconds(.5)
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
                                    .build()
                    );
        }

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}