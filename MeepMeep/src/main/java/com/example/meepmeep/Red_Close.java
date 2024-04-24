package com.example.meepmeep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.ColorScheme;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueLight;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedLight;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class Red_Close {
    static int barcodeInt = 3;
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        RoadRunnerBotEntity redCloseBot = null;

        if (barcodeInt == 2) {
            redCloseBot = new DefaultBotBuilder(meepMeep)
                    .setColorScheme(new ColorSchemeRedLight())
                    // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                    .setConstraints(70, 70, Math.toRadians(197.5918273305535), Math.toRadians(222.7444266666667), 13.26)
                    .followTrajectorySequence(drive ->
                            drive.trajectorySequenceBuilder(new Pose2d(12, -62, Math.toRadians(-90)))
                                    .lineToLinearHeading(new Pose2d(12, -32, Math.toRadians(-90)))
                                    .waitSeconds(.5)
                                    .splineTo(new Vector2d(51, -36), Math.toRadians(0))
                                    .waitSeconds(.5)
                                    .setReversed(true)
                                    .waitSeconds(.5)
                                    .splineTo(new Vector2d(24, -60), Math.toRadians(180))
                                    .splineTo(new Vector2d(-36, -60), Math.toRadians(180))
                                    .splineTo(new Vector2d(-60, -36), Math.toRadians(180))
                                    .waitSeconds(.5)
                                    .setReversed(false)
                                    .splineTo(new Vector2d(-36, -60), Math.toRadians(0))
                                    .splineTo(new Vector2d(24, -60), Math.toRadians(0))
                                    .splineTo(new Vector2d(51, -50), Math.toRadians(75))
                                    .waitSeconds(.5)
                                    .lineToLinearHeading(new Pose2d(48,-60, Math.toRadians(90)))
                                    .build()
                    );
        } else if (barcodeInt ==1) {
            redCloseBot = new DefaultBotBuilder(meepMeep)
                    .setColorScheme(new ColorSchemeRedLight())
                    // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                    .setConstraints(70, 70, Math.toRadians(197.5918273305535), Math.toRadians(222.7444266666667), 13.26)
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
                                    .splineTo(new Vector2d(24, -60), Math.toRadians(180))
                                    .splineTo(new Vector2d(-36, -60), Math.toRadians(180))
                                    .splineTo(new Vector2d(-60, -36), Math.toRadians(180))
                                    .waitSeconds(.5)
                                    .setReversed(false)
                                    .splineTo(new Vector2d(-36, -60), Math.toRadians(0))
                                    .splineTo(new Vector2d(24, -60), Math.toRadians(0))
                                    .splineTo(new Vector2d(51, -50), Math.toRadians(75))
                                    .waitSeconds(.5)
                                    .lineToLinearHeading(new Pose2d(48,-60, Math.toRadians(90)))
                                    .build()
                    );
        } else {
            redCloseBot = new DefaultBotBuilder(meepMeep)
                    .setColorScheme(new ColorSchemeRedLight())
                    // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                    .setConstraints(70, 70, Math.toRadians(197.5918273305535), Math.toRadians(222.7444266666667), 13.26)
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
                                    .splineTo(new Vector2d(24, -60), Math.toRadians(180))
                                    .splineTo(new Vector2d(-36, -60), Math.toRadians(180))
                                    .splineTo(new Vector2d(-60, -36), Math.toRadians(180))
                                    .waitSeconds(.5)
                                    .setReversed(false)
                                    .splineTo(new Vector2d(-36, -60), Math.toRadians(0))
                                    .splineTo(new Vector2d(24, -60), Math.toRadians(0))
                                    .splineTo(new Vector2d(51, -50), Math.toRadians(75))
                                    .waitSeconds(.5)
                                    .lineToLinearHeading(new Pose2d(48,-60, Math.toRadians(90)))
                                    .build()
                    );
        }

        RoadRunnerBotEntity redFarBot = null;
        if (barcodeInt ==2) {
            redFarBot = new DefaultBotBuilder(meepMeep)
                    .setColorScheme(new ColorSchemeRedDark())
                    // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                    .setConstraints(70, 70, Math.toRadians(197.5918273305535), Math.toRadians(222.7444266666667), 13.26)
                    .followTrajectorySequence(drive ->
                            drive.trajectorySequenceBuilder(new Pose2d(-36, -62, Math.toRadians(-90)))
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
                                    .splineTo(new Vector2d(51, -22), Math.toRadians(-75))
                                    .waitSeconds(.5)
                                    .lineToLinearHeading(new Pose2d(48,-12, Math.toRadians(-90)))
                                    .build()
                    );
        } else if (barcodeInt == 1) {
            redFarBot = new DefaultBotBuilder(meepMeep)
                    .setColorScheme(new ColorSchemeRedDark())
                    // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                    .setConstraints(70, 70, Math.toRadians(197.5918273305535), Math.toRadians(222.7444266666667), 13.26)
                    .followTrajectorySequence(drive ->
                            drive.trajectorySequenceBuilder(new Pose2d(-36, -62, Math.toRadians(-90)))
                                    .setReversed(true)
                                    .splineTo(new Vector2d(-40, -36), Math.toRadians(135))
                                    .setReversed(false)
                                    .waitSeconds(.5)
                                    .strafeLeft(15)
                                    .setReversed(true)
                                    .splineTo(new Vector2d(-60,-12), Math.toRadians(180))
                                    .setReversed(false)
                                    .waitSeconds(.5)
                                    .splineTo(new Vector2d(24, -12), Math.toRadians(0))
                                    .splineTo(new Vector2d(51, -30), Math.toRadians(0))
                                    .setReversed(true)
                                    .waitSeconds(.5)
                                    .splineTo(new Vector2d(24, -12), Math.toRadians(180))
                                    .splineTo(new Vector2d(-60, -12), Math.toRadians(180))
                                    .waitSeconds(.5)
                                    .setReversed(false)
                                    .splineTo(new Vector2d(24, -12), Math.toRadians(0))
                                    .splineTo(new Vector2d(51, -22), Math.toRadians(-75))
                            .waitSeconds(.5)
                            .lineToLinearHeading(new Pose2d(48,-12, Math.toRadians(-90)))
                            .build()
                    );
        } else {
            redFarBot = new DefaultBotBuilder(meepMeep)
                    .setColorScheme(new ColorSchemeRedDark())
                    // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                    .setConstraints(70, 70, Math.toRadians(197.5918273305535), Math.toRadians(222.7444266666667), 13.26)
                    .followTrajectorySequence(drive ->
                            drive.trajectorySequenceBuilder(new Pose2d(-36, -62, Math.toRadians(-90)))
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
                                    .splineTo(new Vector2d(51, -22), Math.toRadians(-75))
                                    .waitSeconds(.5)
                                    .lineToLinearHeading(new Pose2d(48,-12, Math.toRadians(-90)))
                                    .build()
                    );
        }

        RoadRunnerBotEntity blueFarBot = null;
        if (barcodeInt ==2) {
            blueFarBot = new DefaultBotBuilder(meepMeep)
                    .setColorScheme(new ColorSchemeBlueDark())
                    // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                    .setConstraints(70, 70, Math.toRadians(197.5918273305535), Math.toRadians(222.7444266666667), 13.26)
                    .followTrajectorySequence(drive ->
                            drive.trajectorySequenceBuilder(new Pose2d(-36, 62, Math.toRadians(90)))
                                    .lineToLinearHeading(new Pose2d(-36, 32, Math.toRadians(90)))
                                    .waitSeconds(.5)
                                    .lineToLinearHeading(new Pose2d(-50, 32, Math.toRadians(90)))
                                    .setReversed(true)
                                    .splineTo(new Vector2d(-60,12), Math.toRadians(180))
                                    .setReversed(false)
                                    .waitSeconds(.5)
                                    .splineTo(new Vector2d(24, 12), Math.toRadians(0))
                                    .splineTo(new Vector2d(51, 36), Math.toRadians(0))
                                    .setReversed(true)
                                    .waitSeconds(.5)
                                    .splineTo(new Vector2d(24, 12), Math.toRadians(180))
                                    .splineTo(new Vector2d(-60, 12), Math.toRadians(180))
                                    .waitSeconds(.5)
                                    .setReversed(false)
                                    .splineTo(new Vector2d(24, 12), Math.toRadians(0))
                                    .splineTo(new Vector2d(51, 22), Math.toRadians(75))
                                    .waitSeconds(.5)
                                    .lineToLinearHeading(new Pose2d(48,12, Math.toRadians(90)))
                                    .build()
                    );
        } else if (barcodeInt == 3) {
            blueFarBot = new DefaultBotBuilder(meepMeep)
                    .setColorScheme(new ColorSchemeBlueDark())
                    // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                    .setConstraints(70, 70, Math.toRadians(197.5918273305535), Math.toRadians(222.7444266666667), 13.26)
                    .followTrajectorySequence(drive ->
                            drive.trajectorySequenceBuilder(new Pose2d(-36, 62, Math.toRadians(90)))
                                    .setReversed(true)
                                    .splineTo(new Vector2d(-40, 36), Math.toRadians(-135))
                                    .setReversed(false)
                                    .waitSeconds(.5)
                                    .strafeRight(15)
                                    .setReversed(true)
                                    .splineTo(new Vector2d(-60,12), Math.toRadians(180))
                                    .setReversed(false)
                                    .waitSeconds(.5)
                                    .splineTo(new Vector2d(24, 12), Math.toRadians(0))
                                    .splineTo(new Vector2d(51, 30), Math.toRadians(0))
                                    .setReversed(true)
                                    .waitSeconds(.5)
                                    .splineTo(new Vector2d(24, 12), Math.toRadians(180))
                                    .splineTo(new Vector2d(-60, 12), Math.toRadians(180))
                                    .waitSeconds(.5)
                                    .setReversed(false)
                                    .splineTo(new Vector2d(24, 12), Math.toRadians(0))
                                    .splineTo(new Vector2d(51, 22), Math.toRadians(75))
                                    .waitSeconds(.5)
                                    .lineToLinearHeading(new Pose2d(48,12, Math.toRadians(90)))
                                    .build()
                    );
        } else {
            blueFarBot = new DefaultBotBuilder(meepMeep)
                    .setColorScheme(new ColorSchemeBlueDark())
                    // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                    .setConstraints(70, 70, Math.toRadians(197.5918273305535), Math.toRadians(222.7444266666667), 13.26)
                    .followTrajectorySequence(drive ->
                            drive.trajectorySequenceBuilder(new Pose2d(-36, 62, Math.toRadians(90)))
                                    .setReversed(true)
                                    .splineTo(new Vector2d(-32, 36), Math.toRadians(-45))
                                    .setReversed(false)
                                    .waitSeconds(.5)
                                    .strafeLeft(10)
                                    .setReversed(true)
                                    .splineTo(new Vector2d(-60,12), Math.toRadians(180))
                                    .setReversed(false)
                                    .waitSeconds(.5)
                                    .splineTo(new Vector2d(24, 12), Math.toRadians(0))
                                    .splineTo(new Vector2d(51, 42), Math.toRadians(0))
                                    .setReversed(true)
                                    .waitSeconds(.5)
                                    .splineTo(new Vector2d(24, 12), Math.toRadians(180))
                                    .splineTo(new Vector2d(-60, 12), Math.toRadians(180))
                                    .waitSeconds(.5)
                                    .setReversed(false)
                                    .splineTo(new Vector2d(24, 12), Math.toRadians(0))
                                    .splineTo(new Vector2d(51, 22), Math.toRadians(75))
                                    .waitSeconds(.5)
                                    .lineToLinearHeading(new Pose2d(48,12, Math.toRadians(90)))
                                    .build()
                    );
        }
        RoadRunnerBotEntity blueCloseBot = null;

        if (barcodeInt == 2) {
            blueCloseBot = new DefaultBotBuilder(meepMeep)
                    .setColorScheme(new ColorSchemeBlueLight())
                    // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                    .setConstraints(70, 70, Math.toRadians(197.5918273305535), Math.toRadians(222.7444266666667), 13.26)
                    .followTrajectorySequence(drive ->
                            drive.trajectorySequenceBuilder(new Pose2d(12, 62, Math.toRadians(90)))
                                    .lineToLinearHeading(new Pose2d(12, 32, Math.toRadians(90)))
                                    .waitSeconds(.5)
                                    .splineTo(new Vector2d(51, 36), Math.toRadians(0))
                                    .waitSeconds(.5)
                                    .setReversed(true)
                                    .waitSeconds(.5)
                                    .splineTo(new Vector2d(24, 60), Math.toRadians(180))
                                    .splineTo(new Vector2d(-36, 60), Math.toRadians(180))
                                    .splineTo(new Vector2d(-60, 36), Math.toRadians(180))
                                    .waitSeconds(.5)
                                    .setReversed(false)
                                    .splineTo(new Vector2d(-36, 60), Math.toRadians(0))
                                    .splineTo(new Vector2d(24, 60), Math.toRadians(0))
                                    .splineTo(new Vector2d(51, 50), Math.toRadians(-75))
                                    .waitSeconds(.5)
                                    .lineToLinearHeading(new Pose2d(48,60, Math.toRadians(-90)))
                                    .build()
                    );
        } else if (barcodeInt ==3) {
            blueCloseBot = new DefaultBotBuilder(meepMeep)
                    .setColorScheme(new ColorSchemeBlueLight())
                    // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                    .setConstraints(70, 70, Math.toRadians(197.5918273305535), Math.toRadians(222.7444266666667), 13.26)
                    .followTrajectorySequence(drive ->
                            drive.trajectorySequenceBuilder(new Pose2d(12, 62, Math.toRadians(90)))
                                    .setReversed(true)
                                    .splineTo(new Vector2d(6, 36), Math.toRadians(-135))
                                    .setReversed(false)
                                    .waitSeconds(.5)
                                    .splineTo(new Vector2d(51, 30), Math.toRadians(0))
                                    .waitSeconds(.5)
                                    .setReversed(true)
                                    .waitSeconds(.5)
                                    .splineTo(new Vector2d(24, 60), Math.toRadians(180))
                                    .splineTo(new Vector2d(-36, 60), Math.toRadians(180))
                                    .splineTo(new Vector2d(-60, 36), Math.toRadians(180))
                                    .waitSeconds(.5)
                                    .setReversed(false)
                                    .splineTo(new Vector2d(-36, 60), Math.toRadians(0))
                                    .splineTo(new Vector2d(24, 60), Math.toRadians(0))
                                    .splineTo(new Vector2d(51, 50), Math.toRadians(-75))
                                    .waitSeconds(.5)
                                    .lineToLinearHeading(new Pose2d(48,60, Math.toRadians(-90)))
                                    .build()
                    );
        } else {
            blueCloseBot = new DefaultBotBuilder(meepMeep)
                    .setColorScheme(new ColorSchemeBlueLight())
                    // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                    .setConstraints(70, 70, Math.toRadians(197.5918273305535), Math.toRadians(222.7444266666667), 13.26)
                    .followTrajectorySequence(drive ->
                            drive.trajectorySequenceBuilder(new Pose2d(12, 62, Math.toRadians(90)))
                                    .setReversed(true)
                                    .splineTo(new Vector2d(18, 36), Math.toRadians(-45))
                                    .setReversed(false)
                                    .waitSeconds(.5)
                                    .splineTo(new Vector2d(51, 42), Math.toRadians(0))
                                    .waitSeconds(.5)
                                    .setReversed(true)
                                    .waitSeconds(.5)
                                    .splineTo(new Vector2d(24, 60), Math.toRadians(180))
                                    .splineTo(new Vector2d(-36, 60), Math.toRadians(180))
                                    .splineTo(new Vector2d(-60, 36), Math.toRadians(180))
                                    .waitSeconds(.5)
                                    .setReversed(false)
                                    .splineTo(new Vector2d(-36, 60), Math.toRadians(0))
                                    .splineTo(new Vector2d(24, 60), Math.toRadians(0))
                                    .splineTo(new Vector2d(51, 50), Math.toRadians(-75))
                                    .waitSeconds(.5)
                                    .lineToLinearHeading(new Pose2d(48,60, Math.toRadians(-90)))
                                    .build()
                    );
        }

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(redCloseBot)
                .addEntity(blueCloseBot)
                .addEntity(redFarBot)
                .addEntity(blueFarBot)
                .start();
    }
}