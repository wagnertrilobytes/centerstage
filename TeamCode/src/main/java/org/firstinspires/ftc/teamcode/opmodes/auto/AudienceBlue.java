package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.helpers.Storage;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.vision.ColorVisionAutoBase;
import org.firstinspires.ftc.teamcode.vision.ColourMassDetectionProcessor;
import org.opencv.core.Scalar;

@Autonomous(name = "Audience Blue", group="Audience")
public class AudienceBlue extends ColorVisionAutoBase {
    TrajectorySequence right_trajOne;
    TrajectorySequence right_trajTwo;
    TrajectorySequence left_trajOne;
    TrajectorySequence left_trajTwo;

    TrajectorySequence middle_trajOne;
    TrajectorySequence middle_trajTwo;

    TrajectorySequence stylePoints_traj;
    TrajectorySequence drop_trajTwo;

    @Override
    public void setup() {
        this.lower = new Scalar(40, 100, 100); // the lower hsv threshold for your detection
        this.upper = new Scalar(180, 255, 255); // the upper hsv threshold for your detection
        this.minArea = () -> 2000; // the minimum area for the detection to consider for your prop
        this.left = () -> 213;
        this.right = () -> 426;
        this.name = "Blue";
        robot.setPoseEstimate(startPos);

//        left_trajOne = robot.trajectorySequenceBuilder(startPos)
//                .splineTo(new Vector2d(32, 30), Math.toRadians(180))
//                .splineTo(new Vector2d(14, 32), Math.toRadians(180))
//                .build();

        left_trajOne =robot.trajectorySequenceBuilder(startPos)
                .lineTo(new Vector2d(-46.26, 62.00))
                .splineTo(new Vector2d(-38.52, 35.52), Math.toRadians(264.66))
                .build();


        left_trajTwo = robot.trajectorySequenceBuilder(left_trajOne.end())
                .lineTo(new Vector2d(-56.26, 41.58))
                .splineTo(new Vector2d(-57.32, 0.11), Math.toRadians(268.53))
                .build();


        middle_trajOne  = robot.trajectorySequenceBuilder(startPos)
                .splineTo(new Vector2d(-37.66, 55.48), Math.toRadians(266.42))
                .splineTo(new Vector2d(-38.52, 35.52), Math.toRadians(264.66))
                .build();


        middle_trajTwo = robot.trajectorySequenceBuilder(middle_trajOne.end())
                .lineToLinearHeading(new Pose2d(-37.66, 55.48, Math.toRadians(180)))
                .splineTo(new Vector2d(-53.33, 28.01), Math.toRadians(-83.52))
                .splineTo(new Vector2d(-51.40, -0.97), Math.toRadians(84.63))
                .build();


        /** these are not done. do them */
        right_trajOne = robot.trajectorySequenceBuilder(startPos)
                .lineTo(new Vector2d(25, 61))
                .lineTo(new Vector2d(25, 40))
                .lineToLinearHeading(new Pose2d(10, 36, Math.toRadians(180)))
                .build();
        right_trajTwo = robot.trajectorySequenceBuilder(right_trajOne.end())
                .lineToLinearHeading(new Pose2d(36, 36, Math.toRadians(90)))
                .build();

        stylePoints_traj = robot.trajectorySequenceBuilder(robot.getPoseEstimate())
                .splineTo(new Vector2d(59.12, 3.54), Math.toRadians(1.35))
                .build();

    }

    Pose2d startPos = new Pose2d(-37, 66, Math.toRadians(90.00));
    double INTAKE_POWER = -0.4;

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
        STYLEPOINTS,
        FINISH
    }

    State currentState = State.IDLE;
    Step currentStep = Step.ONE;

    @Override
    public void onStarted(ColourMassDetectionProcessor.Prop detectedProp) {
        currentStep = Step.ONE;
        if (detectedProp.getPosition() == ColourMassDetectionProcessor.PropPositions.LEFT) {
            currentState = State.LEFT;
            robot.followTrajectorySequenceAsync(left_trajOne);
            // the above is a test..
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
            case FINISH:
                if (!robot.isBusy()) {
                    stop();
                }
                break;
            case STYLEPOINTS:
                if (!robot.isBusy()) {
                    currentStep = Step.FINISH;
                }
                break;
            case TWO:
                if (!robot.isBusy()) {
                    currentStep = Step.STYLEPOINTS;
                    robot.followTrajectorySequenceAsync(stylePoints_traj);
                }
                break;
            case ONE:
                if (!robot.isBusy()) {
                    currentStep = Step.TWO;
                    doIntakeSpin();
                    if (currentState == State.RIGHT) robot.followTrajectorySequenceAsync(right_trajTwo);
                    if (currentState == State.MIDDLE) robot.followTrajectorySequenceAsync(middle_trajTwo);
                    if (currentState == State.LEFT) robot.followTrajectorySequenceAsync(left_trajTwo);
                }
                break;
        }
        telemetry.addData("Step", currentStep);
        telemetry.addData("State", currentState);
    }

    public void doIntakeSpin() {
        robot.intake.setPower(INTAKE_POWER);
        sleep(300);
        robot.intake.setPower(0);
    }
}
