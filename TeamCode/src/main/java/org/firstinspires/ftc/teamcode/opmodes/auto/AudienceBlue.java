package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.helpers.Storage;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.CounterRoller;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Slides;
import org.firstinspires.ftc.teamcode.subsystems.SpicyBucketCR;
import org.firstinspires.ftc.teamcode.vision.ColorVisionAutoBase;
import org.firstinspires.ftc.teamcode.vision.ColourMassDetectionProcessor;
import org.opencv.core.Scalar;

@Config
@Autonomous(name = "Audience Blue", group="Audience", preselectTeleOp = "Centerstage: Teleop PizzaBox Lives On")
public class AudienceBlue extends ColorVisionAutoBase {
    TrajectorySequence right_trajOne;
    TrajectorySequence left_trajOne;
    TrajectorySequence middle_trajOne;

    Intake intake = new Intake();
    CounterRoller roller = new CounterRoller();
    @Override
    public void setup() {
        this.lower = new Scalar(40, 100, 100); // the lower hsv threshold for your detection
        this.upper = new Scalar(180, 255, 255); // the upper hsv threshold for your detection
        this.minArea = () -> 4000; // the minimum area for the detection to consider for your prop
        this.left = () -> 213;
        this.right = () -> 426;
        this.name = "Blue";
        robot.setPoseEstimate(startPos);
        intake.init(hardwareMap);

        Storage.currentPose = robot.getPoseEstimate();

        left_trajOne = robot.trajectorySequenceBuilder(startPos)
                .splineTo(new Vector2d(-40.13, 45.36), Math.toRadians(270.00))
                .splineTo(new Vector2d(-34.18, 34.02), Math.toRadians(0))
                .forward(10)
                .back(11)
                .build();

        middle_trajOne = robot.trajectorySequenceBuilder(startPos)
                .lineToConstantHeading(new Vector2d(-37, 34.18))
                .forward(10)
                .back(11)
                .build();

        right_trajOne = robot.trajectorySequenceBuilder(startPos)
                .lineTo(new Vector2d(-46.26, 61))
                .lineTo(new Vector2d(-46.52, 35.52))
                .forward(12)
                .back(14)
                .build();
    }

    Pose2d startPos = new Pose2d(-37, 61, Math.toRadians(270.00));
    static double INTAKE_POWER = 0.4;

    enum State {
        LEFT,
        RIGHT,
        MIDDLE,
        DEFAULT,
        IDLE
    }

    enum Step {
        GOTO_SPIKE_MARK,
        BACK_AWAY_FROM_SPIKE_MARK,
        FINISH
    }

    State currentState = State.IDLE;
    Step currentStep = Step.GOTO_SPIKE_MARK;

    @Override
    public void onStarted(ColourMassDetectionProcessor.Prop detectedProp) {
        currentStep = Step.BACK_AWAY_FROM_SPIKE_MARK;
        switch (detectedProp.getPosition()) {
            case LEFT:
                currentState = State.LEFT;
                robot.followTrajectorySequenceAsync(left_trajOne);
                break;
            case RIGHT:
                currentState = State.RIGHT;
                robot.followTrajectorySequenceAsync(right_trajOne);
                break;
            case UNFOUND:
            case MIDDLE:
                currentState = State.MIDDLE;
                robot.followTrajectorySequenceAsync(middle_trajOne);
                break;
            default:
                currentState = State.DEFAULT;
                break;
        }
    }

    @Override
    public void onStartedColor(ColourMassDetectionProcessor.Prop detectedProp) {;
        // now we can use recordedPropPosition in our auto code to modify where we place the purple and yellow pixels
        switch (currentStep) {
            case FINISH:
                if (!robot.isBusy()) stop();
                break;
            case BACK_AWAY_FROM_SPIKE_MARK:
                if (!robot.isBusy()) {
                    doIntakeSpin();
                    robot.followTrajectorySequence(robot.trajectorySequenceBuilder(robot.getPoseEstimate())
                            .back(10)
                            .build());
                    currentStep = Step.FINISH;
                }
                break;
        }
        telemetry.addData("Step", currentStep);
        telemetry.addData("State", currentState);
    }

    public void doIntakeSpin() {
        intake.setPower(INTAKE_POWER, 1);
        sleep(350);
        intake.stop();
    }
}
