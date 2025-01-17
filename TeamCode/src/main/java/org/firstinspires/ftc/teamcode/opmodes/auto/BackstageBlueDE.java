package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

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

@Config
@Autonomous(name = "Backstage Blue: Downgraded Edition", group="Backstage", preselectTeleOp = "Centerstage: Teleop PizzaBox Lives On")
public class BackstageBlueDE extends ColorVisionAutoBase {
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

        TrajectorySequenceBuilder builder = robot.trajectorySequenceBuilder(startPos);
//        left_trajOne = AutoPath.build(builder, Storage.TRAJECTORIES.BACKSTAGE.BLUE.LEFT.path);

        left_trajOne = robot.trajectorySequenceBuilder(startPos)
                .strafeLeft(10)
                .forward(40)
                .back(24)
                .build();

        middle_trajOne = robot.trajectorySequenceBuilder(startPos)
                .forward(48)
                .back(21)
                .build();

        right_trajOne = robot.trajectorySequenceBuilder(startPos)
                .forward(27)
                .turn(Math.toRadians(-90))
                .forward(12)
                .back(9)
                .strafeLeft(2)
                .build();
    }

    Pose2d startPos = new Pose2d(14, 60, Math.toRadians(270));
    static double INTAKE_POWER = 0.45;

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
        ALIGN_BACKDROP_MIDDLE,
        ALIGN_BACKDROP_RIGHT,
        DROP,
        BACK_INTO_CORNER,
        SCANNING,
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
                if (!robot.isBusy()) {
                    stop();
                }
                break;
            case ALIGN_BACKDROP_MIDDLE:
                if (!robot.isBusy()) {
                    robot.followTrajectorySequenceAsync(robot.trajectorySequenceBuilder(robot.getPoseEstimate())
                            .turn(Math.toRadians(-90))
                            .back(34)
                            .strafeLeft(16)
                            .build());
                    currentStep = Step.DROP;
                }
                break;
            case ALIGN_BACKDROP_LEFT:
                if (!robot.isBusy()) {
                    robot.followTrajectorySequenceAsync(robot.trajectorySequenceBuilder(robot.getPoseEstimate())
                                    .turn(Math.toRadians(-90))
                                    .back(24)
                                    .strafeLeft(24)
                            .build());
                    currentStep = Step.DROP;
                }
                break;
            case ALIGN_BACKDROP_RIGHT:
                if (!robot.isBusy()) {
                    robot.followTrajectorySequenceAsync(robot.trajectorySequenceBuilder(robot.getPoseEstimate())
                            .back(26)
                            .strafeLeft(12)
                            .build());
                    currentStep = Step.DROP;
                }
                break;
            case ALIGN_WITH_BACKDROP:
                if (!robot.isBusy()) {
                    robot.followTrajectorySequenceAsync(robot.trajectorySequenceBuilder(robot.getPoseEstimate())
                                    .back(10)
                            .build());
                    currentStep =
                            (currentState == State.LEFT ? Step.ALIGN_BACKDROP_LEFT :
                            (currentState == State.RIGHT ? Step.ALIGN_BACKDROP_RIGHT :
                            (currentState == State.MIDDLE ? Step.ALIGN_BACKDROP_MIDDLE : Step.DROP)));
//                    currentStep = Step.SCANNING;
                }
                break;
            case BACK_INTO_CORNER:
                if (!robot.getBusy()) {
                    robot.followTrajectorySequenceAsync(robot.trajectorySequenceBuilder(robot.getPoseEstimate())
                            .forward(8)
                            .strafeRight(20)
//                             .strafeLeft(20)
                            .back(4)
                            .build());
                    currentStep = Step.FINISH;
                }
                break;
            case DROP:
                if (!robot.isBusy()) {
                    robot.busy = true;
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
                    robot.busy = false;
                    currentStep = Step.BACK_INTO_CORNER;
                }
                break;
            case BACK_AWAY_FROM_SPIKE_MARK:
                if (!robot.isBusy()) {
                    doIntakeSpin();
                    robot.followTrajectorySequenceAsync(robot.trajectorySequenceBuilder(robot.getPoseEstimate())
                            .back(10)
                            .build());
                    currentStep = Step.ALIGN_WITH_BACKDROP;
//                    stop();
                }
                break;
        }
        telemetry.addData("Step", currentStep);
        telemetry.addData("State", currentState);
        telemetry.update();
    }

    public void doIntakeSpin() {
        intake.setPower(INTAKE_POWER, 1);
        roller.spinForward();
        sleep(350);
        intake.stop();
        roller.stop();
    }
}
