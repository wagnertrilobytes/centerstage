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

        Pose2d startPos = new Pose2d(14, -60, Math.toRadians(90));
        Pose2d midbefDrop = new Pose2d(14, 44, Math.toRadians(180));
        Vector2d midbefDropV = new Vector2d(24, 60);
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints( 60.34900677139974, 19.68109465623948, 3.0706892013549805, 3.0706892013549805, 16.13)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(startPos)
                                .lineTo(new Vector2d(25, -61))
                                .lineTo(new Vector2d(25, -40))
                                .waitSeconds(1)
                                .lineTo(new Vector2d(25, -55))
                                .lineToLinearHeading(new Pose2d(36, -36, Math.toRadians(270)))
                                .waitSeconds(1)
                                .splineTo(new Vector2d(40, -36), Math.toRadians(180))
                                .splineToConstantHeading(new Vector2d(49.5, -36), Math.toRadians(180))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_LIGHT)
                .setDarkMode(false)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}