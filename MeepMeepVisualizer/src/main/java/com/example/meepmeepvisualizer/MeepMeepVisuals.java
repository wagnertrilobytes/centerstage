package com.example.meepmeepvisualizer;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;

public class MeepMeepVisuals {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        Vector2d ENDPOS = new Vector2d(-57.32, 0.11);
        Pose2d ENDPOSE = new Pose2d(-57.32, 0.11, 0);
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(49.06530693660696, 51.78291908330528, Math.toRadians(167.05832), Math.toRadians(167.05832), -9)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder( new Pose2d(14, 60, Math.toRadians(270)))
                                .lineTo(new Vector2d(25, 61))
                                .lineTo(new Vector2d(23, 39))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_LIGHT)
                .setDarkMode(false)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}