package org.firstinspires.ftc.teamcode.opmodes.auto.examples;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.vision.ColorVisionAutoBase;
import org.firstinspires.ftc.teamcode.vision.ColourMassDetectionProcessor;
import org.opencv.core.Scalar;

import java.lang.annotation.Target;
import java.util.Timer;

@Autonomous(name = "Example Color RR Auto (FSM)")
@Disabled
public class ExampleColorRRAuto extends ColorVisionAutoBase {
    TrajectorySequence trajectory;
    Pose2d startPos = new Pose2d(14, 60, Math.toRadians(270));
    @Override
    public void setup() {
        this.lower = new Scalar(40, 100, 100); // the lower hsv threshold for your detection
        this.upper = new Scalar(180, 255, 255); // the upper hsv threshold for your detection
        this.minArea = () -> 8000; // the minimum area for the detection to consider for your prop
        this.left = () -> 213;
        this.right = () -> 426;
        this.name = "Blue";
        robot.setPoseEstimate(startPos);

        trajectory = robot.trajectorySequenceBuilder(startPos)
                .lineTo(new Vector2d(25, 61))
                .lineTo(new Vector2d(25, 40))
                .build();
    }

    enum State {
        DEFAULT,
        IDLE
    }

    enum Step {
        FINISH,
        ONE
    }

    State currentState = State.IDLE;
    Step currentStep = Step.ONE;

    @Override
    public void onStarted(ColourMassDetectionProcessor.Prop detectedProp) {
        currentStep = Step.ONE;
        currentState = State.DEFAULT;
    }

    @Override
    public void onStartedColor(ColourMassDetectionProcessor.Prop detectedProp) {;
        // now we can use recordedPropPosition in our auto code to modify where we place the purple and yellow pixels
        switch (currentStep) {
            case FINISH:
                if (!robot.isBusy()) {
                    stop();
                }
                break;
            case ONE:
                if (!robot.isBusy()) {
                    currentStep = Step.FINISH;
                    robot.followTrajectorySequenceAsync(trajectory);
                }
                break;
        }
        telemetry.addData("Step", currentStep);
        telemetry.addData("State", currentState);
    }
}
