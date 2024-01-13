package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.vision.ColorVisionAutoBase;
import org.firstinspires.ftc.teamcode.vision.ColourMassDetectionProcessor;
import org.opencv.core.Scalar;

import java.lang.annotation.Target;

@Autonomous(name = "Backstage Red", group="Backstage")
public class BackstageRed extends ColorVisionAutoBase {
    TrajectorySequence right_trajOne;
    TrajectorySequence right_trajTwo;
    TrajectorySequence left_trajOne;
    TrajectorySequence left_trajTwo;

    TrajectorySequence middle_trajOne;
    TrajectorySequence middle_trajTwo;

    TrajectorySequence drop_trajOne;
    TrajectorySequence drop_trajTwo;
    Pose2d startPos = new Pose2d(14, -60, Math.toRadians(90));
    double INTAKE_POWER = -0.4;
    @Override
    public void setup() {
        this.lower = new Scalar(0, 100, 100); // the lower hsv threshold for your detection
        this.upper = new Scalar(255, 255, 255); // the upper hsv threshold for your detection
        this.minArea = () -> 2000; // the minimum area for the detection to consider for your prop
        this.left = () -> 213;
        this.right = () -> 426;
        this.name = "Red";
        robot.setPoseEstimate(startPos);

        left_trajOne = robot.trajectorySequenceBuilder(startPos)
                .splineTo(new Vector2d(32, -30), Math.toRadians(180))
                .splineTo(new Vector2d(10, -32), Math.toRadians(180))
                .build();

        left_trajTwo = robot.trajectorySequenceBuilder(left_trajOne.end())
                .lineToLinearHeading(new Pose2d(36, -36, Math.toRadians(270)))
                .build();

        middle_trajOne = robot.trajectorySequenceBuilder(startPos)
                .lineToLinearHeading(new Pose2d(14, -31.75, Math.toRadians(90)))
                .build();

        middle_trajTwo = robot.trajectorySequenceBuilder(middle_trajOne.end())
                .lineToLinearHeading(new Pose2d(14, -45, Math.toRadians(90)))
                .build();
        right_trajOne = robot.trajectorySequenceBuilder(startPos)
                .lineTo(new Vector2d(25, -61))
                .lineTo(new Vector2d(25, -40))
                .build();
        right_trajTwo = robot.trajectorySequenceBuilder(right_trajOne.end())
                .lineTo(new Vector2d(25, -55))
                .lineToLinearHeading(new Pose2d(36, -36, Math.toRadians(270)))
                .build();

        drop_trajOne = robot.trajectorySequenceBuilder(new Pose2d(36, -36, Math.toRadians(270)))
                .splineTo(new Vector2d(40, -36), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(49.5, -36), Math.toRadians(180))
                .build();

    }

    enum State {
        LEFT,
        RIGHT,
        MIDDLE,
        DEFAULT,
        IDLE
    }

    enum Step {
        ONE,
        TWO,
        DROP,
        FINISH
    }

    State currentState = State.IDLE;
    Step currentStep = Step.ONE;

    @Override
    public void onStarted(ColourMassDetectionProcessor.Prop detectedProp) {
        robot.clawLeft.turnToAngle(6);
        robot.clawRight.turnToAngle(6);
        currentStep = Step.ONE;
        if (detectedProp.getPosition() == ColourMassDetectionProcessor.PropPositions.LEFT) {
            currentState = State.LEFT;
            robot.followTrajectorySequenceAsync(left_trajOne);
        } else if(detectedProp.getPosition() == ColourMassDetectionProcessor.PropPositions.RIGHT) {
            currentState = State.RIGHT;
            robot.followTrajectorySequenceAsync(right_trajOne);
        } else if(detectedProp.getPosition() == ColourMassDetectionProcessor.PropPositions.MIDDLE) {
            currentState = State.MIDDLE;
            robot.followTrajectorySequenceAsync(middle_trajOne);
        } else {
            currentState = State.DEFAULT;
        }
    }

    @Override
    public void onStartedColor(ColourMassDetectionProcessor.Prop detectedProp) {;
        // now we can use recordedPropPosition in our auto code to modify where we place the purple and yellow pixels
        switch (currentStep) {
            case DROP:
                if (!robot.isBusy()) {
                    currentStep = Step.FINISH;
                    double clawSpeed = 0.75;
                    robot.slideLeft.setPower(-0.75);
                    robot.slideRight.setPower(0.75);
                    sleep(850);
                    robot.slideLeft.setPower(0);
                    robot.slideRight.setPower(0);
                    sleep(250);
                    robot.clawLeft.turnToAngle(robot.clawLeft.max - 15);
                    robot.clawRight.turnToAngle(robot.clawRight.max - 15);
//                    robot.followTrajectorySequenceAsync(drop_trajTwo);
                }
                break;
            case TWO:
                if (!robot.isBusy()) {
                    currentStep = Step.DROP;
                    robot.followTrajectorySequenceAsync(drop_trajOne);
                }
                break;
            case ONE:
                if (!robot.isBusy()) {
                    currentStep = Step.TWO;
                    if (currentState == State.RIGHT) {
                        doIntakeSpin();
                        robot.followTrajectorySequenceAsync(right_trajTwo);
                    }
                    if (currentState == State.MIDDLE) {
                        doIntakeSpin();
                        robot.followTrajectorySequenceAsync(middle_trajTwo);
                    }
                    if (currentState == State.LEFT) {
                        doIntakeSpin();
                        robot.followTrajectorySequenceAsync(left_trajTwo);
                    }
                }
                break;
        }
        telemetry.addData("Step", currentStep);
        telemetry.addData("State", currentState);
    }

    public void doIntakeSpin() {
        robot.intake.setPower(-INTAKE_POWER);
        sleep(300);
        robot.intake.setPower(0);
    }
}
