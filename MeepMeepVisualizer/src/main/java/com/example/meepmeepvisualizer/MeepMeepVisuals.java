package com.example.meepmeepvisualizer;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepVisuals {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(49.06530693660696, 51.78291908330528, Math.toRadians(167.05832), Math.toRadians(167.05832), -9)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(11.26, -62.90, Math.toRadians(90.00)))
                        .UNSTABLE_addTemporalMarkerOffset(17.76,() -> {})
                        .splineTo(new Vector2d(11.26, -24.66), Math.toRadians(91.17))
                        .splineTo(new Vector2d(35.26, -37.90), Math.toRadians(267.06))
                        .splineTo(new Vector2d(12.08, 13.08), Math.toRadians(-81.71))
                        .splineTo(new Vector2d(45.85, -35.42), Math.toRadians(178.60))
                        .splineTo(new Vector2d(58.26, 61.57), Math.toRadians(82.71))
                        .splineTo(new Vector2d(32.94, 60.25), Math.toRadians(182.99))
                        .splineTo(new Vector2d(7.94, 58.59), Math.toRadians(183.79))
                        .splineTo(new Vector2d(9.60, 39.39), Math.toRadians(0.00))
                        .splineTo(new Vector2d(31.94, 40.72), Math.toRadians(-18.43))
                        .splineTo(new Vector2d(60.91, 17.88), Math.toRadians(270.00))
                        .splineTo(new Vector2d(25.99, 13.74), Math.toRadians(186.76))
                        .splineTo(new Vector2d(11.26, 16.39), Math.toRadians(169.81))
                        .splineTo(new Vector2d(-13.57, 9.93), Math.toRadians(194.57))
                        .splineTo(new Vector2d(-32.44, 12.25), Math.toRadians(173.00))
                        .splineTo(new Vector2d(-36.25, -3.97), Math.toRadians(256.79))
                        .splineTo(new Vector2d(1.32, -10.26), Math.toRadians(-9.50))
                        .splineTo(new Vector2d(23.50, -6.62), Math.toRadians(9.32))
                        .splineTo(new Vector2d(64.22, -13.57), Math.toRadians(-9.69))
                        .splineTo(new Vector2d(59.59, -36.58), Math.toRadians(258.61))
                        .splineTo(new Vector2d(61.41, -57.43), Math.toRadians(243.43))
                        .splineTo(new Vector2d(34.43, -57.60), Math.toRadians(90.00))
                        .splineTo(new Vector2d(33.93, -32.77), Math.toRadians(175.68))
                        .splineTo(new Vector2d(7.45, -30.79), Math.toRadians(175.71))
                        .splineTo(new Vector2d(13.08, -51.48), Math.toRadians(-74.78))
                        .splineTo(new Vector2d(-9.60, -54.12), Math.toRadians(90.00))
                        .splineTo(new Vector2d(-17.21, -35.26), Math.toRadians(111.97))
                        .splineTo(new Vector2d(-40.06, -31.12), Math.toRadians(169.73))
                        .splineTo(new Vector2d(-35.75, -44.52), Math.toRadians(-72.20))
                        .splineTo(new Vector2d(-41.21, -57.10), Math.toRadians(246.53))
                        .splineTo(new Vector2d(-61.41, -52.63), Math.toRadians(90.00))
                        .splineTo(new Vector2d(-4.14, 49.32), Math.toRadians(60.68))
                        .splineTo(new Vector2d(-38.23, 54.29), Math.toRadians(171.71))
                        .splineTo(new Vector2d(-54.62, 56.28), Math.toRadians(173.09))
                        .splineTo(new Vector2d(-52.97, 42.37), Math.toRadians(-83.21))
                        .splineTo(new Vector2d(-4.97, 28.63), Math.toRadians(-53.53))
                        .splineTo(new Vector2d(24.66, 29.46), Math.toRadians(1.60))
                        .splineTo(new Vector2d(-1.66, -30.46), Math.toRadians(246.29))
                        .splineTo(new Vector2d(-31.45, -41.05), Math.toRadians(199.57))
                        .splineTo(new Vector2d(-34.10, 26.48), Math.toRadians(92.25))
                        .splineTo(new Vector2d(-57.60, 13.57), Math.toRadians(90.00))
                        .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_LIGHT)
                .setDarkMode(false)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}