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
@Autonomous(name = "Backstage Blurfytgue", group="Backstage")
public class BackstageBlue extends ColorVisionAutoBase {
    TrajectorySequence right_trajOne;
    TrajectorySequence right_trajTwo;
    TrajectorySequence left_trajOne;
    TrajectorySequence left_trajTwo;

    TrajectorySequence middle_trajOne;
    TrajectorySequence middle_trajTwo;

    TrajectorySequence drop_trajOne;
    TrajectorySequence drop_trajTwo;
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
                .lineTo(new Vector2d(25, 61))
                .lineTo(new Vector2d(23, 39))
                .strafeRight(5)
                .build();

        middle_trajOne = robot.trajectorySequenceBuilder(startPos)
                .lineToConstantHeading(new Vector2d(14, 34))
                .forward(20)
                .back(18)
                .build();

        right_trajOne = robot.trajectorySequenceBuilder(startPos)
                .lineTo(new Vector2d(20, 40))
                .splineTo(new Vector2d(10, 35), Math.toRadians(200))
                .forward(2)
                .build();
    }

    Pose2d startPos = new Pose2d(14, 60, Math.toRadians(270));
    Pose2d midbefDrop = new Pose2d(14, 44, Math.toRadians(180));
    Vector2d midbefDropV = new Vector2d(20, 60);
    static double INTAKE_POWER = 0.4;

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
        currentStep = Step.ONE;
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
            case DROP:
                if (!robot.isBusy()) {
                    currentStep = Step.FINISH;
                    slides.setPower(slideSpeed);
                    sleep(2275);
                    slides.stop();
                    bucketCR.setSlideArmPower(-bArmSpeed);
                    sleep(150);
                    bucketCR.setSlideArmPower(0);
                    bucketCR.setWheelPower(-1);
                    bucketCR.setBucketArmPower(-0.3);
                    sleep(150);
                    bucketCR.setBucketArmPower(0);
                    sleep(1500);
                    bucketCR.stop();
                }
                break;
            case TWO:
                if (!robot.isBusy()) {
                    if (currentState == State.LEFT) {
                        robot.followTrajectorySequence(robot.trajectorySequenceBuilder(robot.getPoseEstimate())
                                        .forward(2)
                                        .strafeLeft(2)
                                .build());
                    }
                    drop_trajOne = robot.trajectorySequenceBuilder(robot.getPoseEstimate())
                            .lineToLinearHeading(new Pose2d(37, 36, Math.toRadians(180)))
                            .build();
                    currentStep = Step.DROP;
                    robot.followTrajectorySequenceAsync(drop_trajOne);
                }
                break;
            case ONE:
                if (!robot.isBusy()) {
                    currentStep = Step.TWO;
                    doIntakeSpin();
                    if (currentState == State.MIDDLE) {
                        robot.followTrajectorySequenceAsync(robot.trajectorySequenceBuilder(robot.getPoseEstimate())
                                .back(10)
                                .build());
                    }
                    if (currentState == State.RIGHT) {
                        robot.followTrajectorySequenceAsync(robot.trajectorySequenceBuilder(robot.getPoseEstimate())
                                .back(3)
                                        .strafeLeft(3)
                                .build());
                    }
                }
                break;
        }
        telemetry.addData("Step", currentStep);
        telemetry.addData("State", currentState);
    }

    public void doIntakeSpin() {
        intake.setPower(INTAKE_POWER, 1);
        roller.spinForward();
        sleep(325);
        intake.stop();
        roller.stop();
    }
}
