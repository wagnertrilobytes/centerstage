package com.example.meepmeepvisualizer;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;

public class MeepMeepVisuals {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(500);

        Pose2d startPos = new Pose2d(-37, -61, Math.toRadians(90));

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints( 45.14696128158258, 19.68109465623948, 2.49843921661377, 2.49843921661377, 16.13)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(startPos)
                                // \/ TRAJ ONE
                                .lineTo(new Vector2d(-40.13, -45.36))
                                .lineTo(new Vector2d(-34.18, -34.02))
                                .turn(Math.toRadians(-90))
                                .forward(10)
                                .back(11)
//                                .waitSeconds(1) // \/ GOTO_DROP
//                                .lineToLinearHeading(new Pose2d(37, 36, Math.toRadians(180)))
//                                .waitSeconds(1) // \/ ALIGNMENT/DROP
////                                .lineToLinearHeading(new Pose2d(47,36, Math.toRadians(180)))
//                                .back(10)


                                // OPTIONAL: ONLY IF LEFT OR RIGHT

                                // MIDDLE ALIGNMENT
//                                .strafeLeft(2)

                                // LEFT ALIGNMENT
//                                .strafeRight(4)

                                // RIGHT ALIGNMENT
//                                .strafeLeft(5)

//                                .waitSeconds(1) // \/ BACK_INTO_CORNER
//                                .forward(10)
//                                .strafeRight(24)
//                                .back(10)

                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_LIGHT)
                .setDarkMode(false)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}