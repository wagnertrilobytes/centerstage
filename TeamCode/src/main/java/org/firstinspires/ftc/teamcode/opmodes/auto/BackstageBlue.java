package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.teamcode.helpers.Storage;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.subsystems.CounterRoller;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Slides;
import org.firstinspires.ftc.teamcode.subsystems.SpicyBucketCR;
import org.firstinspires.ftc.teamcode.vision.ColorVisionAutoBase;
import org.firstinspires.ftc.teamcode.vision.ColourMassDetectionProcessor;
import org.opencv.core.Scalar;

import java.lang.annotation.Target;
@Config
@Autonomous(name = "Backstage Blue", group="Backstage", preselectTeleOp = "Centerstage: Teleop PizzaBox Lives On")
public class BackstageBlue extends ColorVisionAutoBase {
    TrajectorySequence right_trajOne;
    TrajectorySequence left_trajOne;

    TrajectorySequence middle_trajOne;

    TrajectorySequence drop_trajOne;
    Intake intake = new Intake();
    SpicyBucketCR bucketCR = new SpicyBucketCR();
    Slides slides = new Slides();
    CounterRoller roller = new CounterRoller();

    public static double bArmSpeed = 1;
    public static double bBucSpeed = 0.75;
    public static double slideSpeed = 1;
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
        bucketCR.init(hardwareMap);
        slides.init(hardwareMap);
        roller.init(hardwareMap);
        Storage.currentPose = robot.getPoseEstimate();

        left_trajOne =robot.trajectorySequenceBuilder(startPos)
                .lineTo(new Vector2d(23, 60))
                .lineTo(new Vector2d(23, 39))
                .forward(10)
                .back(11)
                .build();

        middle_trajOne = robot.trajectorySequenceBuilder(startPos)
                .lineToConstantHeading(new Vector2d(14, 34))
                .forward(20)
                .back(15)
                .build();

        right_trajOne = robot.trajectorySequenceBuilder(startPos)
                .lineTo(new Vector2d(14, 40))
                .splineTo(new Vector2d(10, 35), Math.toRadians(180))
                .forward(5)
                .back(6)
                .build();
    }

    Pose2d startPos = new Pose2d(14, 60, Math.toRadians(270));
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
        GOTO_DROP,
        ALIGN_WITH_BACKDROP,
        ALIGN_BACKDROP_LEFT,
        ALIGN_BACKDROP_RIGHT,
        DROP,
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
            case ALIGN_BACKDROP_LEFT:
                if (!robot.isBusy()) {
                    robot.followTrajectorySequenceAsync(robot.trajectorySequenceBuilder(robot.getPoseEstimate())
                            .strafeRight(4)
                            .build());
                    currentStep = Step.DROP;
                }
                break;
            case ALIGN_BACKDROP_RIGHT:
                if (!robot.isBusy()) {
                    robot.followTrajectorySequenceAsync(robot.trajectorySequenceBuilder(robot.getPoseEstimate())
                            .strafeLeft(5)
                            .build());
                    currentStep = Step.DROP;
                }
                break;
            case ALIGN_WITH_BACKDROP:
                if (!robot.isBusy()) {
                    robot.followTrajectorySequenceAsync(robot.trajectorySequenceBuilder(robot.getPoseEstimate())
                                    .lineToLinearHeading(new Pose2d(49,36, Math.toRadians(180)))
                            .build());
                    currentStep =
                            (currentState == State.LEFT ? Step.ALIGN_BACKDROP_LEFT :
                            (currentState == State.RIGHT ? Step.ALIGN_BACKDROP_RIGHT : Step.DROP));
                }
                break;
            case DROP:
                if (!robot.isBusy()) {
                    slides.setPower(slideSpeed);
                    sleep(2300);
                    slides.stop();
                    bucketCR.setSlideArmPower(-bArmSpeed);
                    sleep(450);
                    bucketCR.setSlideArmPower(0);
                    bucketCR.setBucketArmPower(-0.3);
                    sleep(150);
                    bucketCR.setWheelPower(-1);
                    bucketCR.setBucketArmPower(0);
                    sleep(1500);
                    bucketCR.stop();
                    Storage.currentPose = robot.getPoseEstimate();
                    currentStep = Step.FINISH;
                }
                break;
            case GOTO_DROP:
                if (!robot.isBusy()) {
                    drop_trajOne = robot.trajectorySequenceBuilder(robot.getPoseEstimate())
                            .lineToLinearHeading(new Pose2d(37, 36, Math.toRadians(180)))
                            .build();
                    currentStep = Step.ALIGN_WITH_BACKDROP;
                    robot.followTrajectorySequenceAsync(drop_trajOne);
                }
                break;
            case BACK_AWAY_FROM_SPIKE_MARK:
                if (!robot.isBusy()) {
                    doIntakeSpin();
                    robot.followTrajectorySequence(robot.trajectorySequenceBuilder(robot.getPoseEstimate())
                            .back(10)
                            .build());
                    currentStep = Step.GOTO_DROP;
                }
                break;
        }
        telemetry.addData("Step", currentStep);
        telemetry.addData("State", currentState);
    }

    public void doIntakeSpin() {
        intake.setPower(INTAKE_POWER, 1);
        roller.spinForward();
        sleep(350);
        intake.stop();
        roller.stop();
    }
}
