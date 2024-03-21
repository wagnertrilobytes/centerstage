package org.firstinspires.ftc.teamcode.opmodes.auto;

import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequenceBuilder;

public class AutoPath {
    AutoDirection direction;
    double amt;
    public AutoPath(AutoDirection dir, double amt) {
        this.direction = dir;
        this.amt = amt;
    }
    public enum AutoDirection {
        FORWARD, LEFT, RIGHT, BACK, TURN_DEG, TURN_RAD
    }
    public static TrajectorySequence build(TrajectorySequenceBuilder builder, AutoPath[] arr) {
        for(AutoPath path: arr) {
            switch(path.direction) {
                case FORWARD:
                    builder = builder.forward(path.amt);
                    break;
                case BACK:
                    builder = builder.back(path.amt);
                    break;
                case LEFT:
                    builder = builder.strafeLeft(path.amt);
                    break;
                case RIGHT:
                    builder = builder.strafeRight(path.amt);
                    break;
                case TURN_DEG:
                    builder = builder.turn(Math.toRadians(path.amt));
                    break;
                case TURN_RAD:
                    builder = builder.turn(path.amt);
                    break;
            }
        }
        return builder.build();
    }
}

