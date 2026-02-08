package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.ParallelAction;
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

        bot.runAction(bot.getDrive().actionBuilder(new Pose2d(-63.3, 15.2, Math.toRadians(0)))
                .strafeToLinearHeading(new Vector2d(-58, 15), Math.toRadians(20))
                .setTangent(Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(-54, 25, Math.toRadians(90)), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-54, 60), Math.toRadians(280))
                .splineToConstantHeading(new Vector2d(-46, 25), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-46, 60), Math.toRadians(90))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(bot)
                .start();
    }
}