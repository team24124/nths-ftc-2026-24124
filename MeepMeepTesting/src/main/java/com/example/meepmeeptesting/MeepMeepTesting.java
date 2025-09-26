package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity bot = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeBlueDark())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 16)
                .build();

        // Assuming spindexer & no turret
        bot.runAction(bot.getDrive().actionBuilder(new Pose2d(60, -20, Math.toRadians(180)))
                        .strafeToSplineHeading(new Vector2d(57, -20), Math.toRadians(195))
                        .waitSeconds(1)

                        .strafeToSplineHeading(new Vector2d(41.5, -27), Math.toRadians(270))
                        .splineToConstantHeading(new Vector2d(35.5, -47), Math.toRadians(270))

                        .strafeToSplineHeading(new Vector2d(20, -25), Math.toRadians(220))
                        .splineToConstantHeading(new Vector2d(-5, -15), Math.toRadians(180))

                        .strafeToSplineHeading(new Vector2d(5, -25), Math.toRadians(270))
                        .splineToConstantHeading(new Vector2d(12, -47), Math.toRadians(270))

                        .strafeToSplineHeading(new Vector2d(5, -33), Math.toRadians(220))
                        .splineToConstantHeading(new Vector2d(-10, -18), Math.toRadians(180))

                        .strafeToSplineHeading(new Vector2d(-10.5, -20), Math.toRadians(270))
                        .splineToConstantHeading(new Vector2d(-12, -47), Math.toRadians(270))

                        .strafeToSplineHeading(new Vector2d(-29, -34), Math.toRadians(220))
                        .build());

        // Assuming spindexer & turret
        bot.runAction(bot.getDrive().actionBuilder(new Pose2d(60, -20, Math.toRadians(270)))
                .strafeToSplineHeading(new Vector2d(41.5, -27), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(35.5, -47), Math.toRadians(270))

                .strafeToSplineHeading(new Vector2d(20, -25), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(-5, -15), Math.toRadians(180))

                .strafeToSplineHeading(new Vector2d(5, -25), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(12, -47), Math.toRadians(270))

                .strafeToSplineHeading(new Vector2d(5, -33), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(-10, -18), Math.toRadians(180))

                .strafeToSplineHeading(new Vector2d(-10.5, -20), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(-12, -47), Math.toRadians(270))

                .strafeToSplineHeading(new Vector2d(-29, -34), Math.toRadians(270))
                .build());

        // Assuming no spindexer & no turret & GPP
        bot.runAction(bot.getDrive().actionBuilder(new Pose2d(60, -20, Math.toRadians(180)))
                .strafeToSplineHeading(new Vector2d(57, -20), Math.toRadians(195))
                .waitSeconds(1)

                .strafeToSplineHeading(new Vector2d(41.5, -27), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(35.5, -47), Math.toRadians(270))

                .strafeToSplineHeading(new Vector2d(20, -25), Math.toRadians(220))
                .splineToConstantHeading(new Vector2d(-5, -15), Math.toRadians(180))

                .strafeToSplineHeading(new Vector2d(5, -25), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(12, -47), Math.toRadians(270))

                .strafeToSplineHeading(new Vector2d(5, -33), Math.toRadians(220))
                .splineToConstantHeading(new Vector2d(-10, -18), Math.toRadians(180))

                .strafeToSplineHeading(new Vector2d(-10.5, -20), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(-12, -47), Math.toRadians(270))

                .strafeToSplineHeading(new Vector2d(-29, -34), Math.toRadians(220))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(bot)
                .start();
    }
}