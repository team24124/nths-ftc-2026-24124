package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedLight;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.util.Arrays;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800, 70);

        RoadRunnerBotEntity bot = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeRedLight())
                .setConstraints(50, 50, Math.toRadians(180), Math.toRadians(180), 16)
                .build();

        bot.runAction(bot.getDrive().actionBuilder(new Pose2d(53, -48, Math.toRadians(306)))
                        .setTangent(Math.toRadians(135))
                        .splineToLinearHeading(new Pose2d(30, -21, Math.toRadians(306)), Math.toRadians(135), new TranslationalVelConstraint(90), new ProfileAccelConstraint(-60, 90))

                        .setTangent(Math.toRadians(135))
                        .splineToSplineHeading(new Pose2d(13, -30, Math.toRadians(270)), Math.toRadians(270), new MinVelConstraint(Arrays.asList(
                                new TranslationalVelConstraint(40),
                                new AngularVelConstraint(Math.PI / 2))))
                        .splineToConstantHeading(new Vector2d(12, -48), Math.toRadians(90), new TranslationalVelConstraint(7))



                        .splineToSplineHeading(new Pose2d(3, -46, Math.toRadians(30)), Math.toRadians(245))
                        .splineToConstantHeading(new Vector2d(1, -52), Math.toRadians(270))
                        .waitSeconds(2)



                        .strafeToSplineHeading(new Vector2d(15, -16), Math.toRadians(320), new TranslationalVelConstraint(90), new ProfileAccelConstraint(-60, 90))
                        .waitSeconds(1)

                        .strafeToSplineHeading(new Vector2d(-5, -25), Math.toRadians(270), new MinVelConstraint(Arrays.asList(
                                new TranslationalVelConstraint(40),
                                new AngularVelConstraint(Math.PI / 2))))
                        .splineToConstantHeading(new Vector2d(-12, -50), Math.toRadians(75), new TranslationalVelConstraint(8))

                        .splineToSplineHeading(new Pose2d(-5, -33, Math.toRadians(320)), Math.toRadians(58), new TranslationalVelConstraint(90), new ProfileAccelConstraint(-60, 90))
                        .splineToConstantHeading(new Vector2d(15, -16), Math.toRadians(0), new TranslationalVelConstraint(90), new ProfileAccelConstraint(-60, 90))
                        .waitSeconds(1)

                        .strafeToSplineHeading(new Vector2d(-25.5, -27), Math.toRadians(270), new MinVelConstraint(Arrays.asList(
                                new TranslationalVelConstraint(40),
                                new AngularVelConstraint(Math.PI / 2))))
                        .splineToConstantHeading(new Vector2d(-35.5, -52), Math.toRadians(270), new TranslationalVelConstraint(8))

                        .strafeToSplineHeading(new Vector2d(40, -15),  Math.toRadians(300), new TranslationalVelConstraint(90), new ProfileAccelConstraint(-60, 90))
                        .waitSeconds(2)
                        .build());

//        bot.runAction(bot.getDrive().actionBuilder(new Pose2d(-63, -16, Math.toRadians(0)))
//                        .strafeToSplineHeading(new Vector2d(-54, -16), Math.toRadians(338), new TranslationalVelConstraint(90), new ProfileAccelConstraint(-60, 90))
//                        .waitSeconds(1)
//
//                        .strafeToSplineHeading(new Vector2d(-40, -29), Math.toRadians(270), new MinVelConstraint(Arrays.asList(
//                                new TranslationalVelConstraint(40),
//                                new AngularVelConstraint(Math.PI / 2))))
//                        .splineToConstantHeading(new Vector2d(-35.5, -52), Math.toRadians(90), new TranslationalVelConstraint(8))
//
//                        .splineToSplineHeading(new Pose2d(-54, -16, Math.toRadians(338)), Math.toRadians(115), new TranslationalVelConstraint(90), new ProfileAccelConstraint(-60, 90))
//                        .waitSeconds(1)
//
//                        .strafeToLinearHeading(new Vector2d(-40, -60), Math.toRadians(330), new TranslationalVelConstraint(90), new ProfileAccelConstraint(-60, 90))
//                        .waitSeconds(5)
//
//                        .setTangent(Math.toRadians(125))
//                        .strafeToSplineHeading(new Vector2d(-54, -16), Math.toRadians(338), new TranslationalVelConstraint(90), new ProfileAccelConstraint(-60, 90))
//                        .waitSeconds(1)
//
//                        .setTangent(Math.toRadians(300))
//                        .splineToSplineHeading(new Pose2d(-46, -58, Math.toRadians(210)), Math.toRadians(210), new TranslationalVelConstraint(75), new ProfileAccelConstraint(-60, 90))
//                        .splineToSplineHeading(new Pose2d(-60, -62, Math.toRadians(180)), Math.toRadians(180), new TranslationalVelConstraint(8))
//                        .waitSeconds(1)
//
//                        .strafeToConstantHeading(new Vector2d(-50, -50), new TranslationalVelConstraint(90), new ProfileAccelConstraint(-60, 90))
//                        .splineToSplineHeading(new Pose2d(-54, -16, Math.toRadians(338)), Math.toRadians(130), new TranslationalVelConstraint(90), new ProfileAccelConstraint(-60, 90))
//                        .waitSeconds(1)
//                        .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(bot)
                .start();
    }
}