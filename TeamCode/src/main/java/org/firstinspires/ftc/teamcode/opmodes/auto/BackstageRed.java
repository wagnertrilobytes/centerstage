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
@Autonomous(name = "Backstage Redrfytgue", group="Backstage")
public class BackstageRed extends ColorVisionAutoBase {
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
        this.lower = new Scalar(0, 100, 100); // the lower hsv threshold for your detection
        this.upper = new Scalar(6, 255, 255); // the upper hsv threshold for your detection
        this.minArea = () -> 4000; // the minimum area for the detection to consider for your prop
        this.left = () -> 213;
        this.right = () -> 426;
        this.name = "Red";
        robot.setPoseEstimate(startPos);
        intake.init(hardwareMap);
        bucketCR.init(hardwareMap);
        slides.init(hardwareMap);
        roller.init(hardwareMap);
        Storage.currentPose = robot.getPoseEstimate();

//        left_trajOne = robot.trajectorySequenceBuilder(startPos)
//                .splineTo(new Vector2d(32, 30), Math.toRadians(180))
//                .splineTo(new Vector2d(14, 32), Math.toRadians(180))
//                .build();

        left_trajOne = robot.trajectorySequenceBuilder(startPos)
                .splineTo(new Vector2d(32, -30), Math.toRadians(180))
                .splineTo(new Vector2d(10, -32), Math.toRadians(180))
                .build();

        left_trajTwo = robot.trajectorySequenceBuilder(left_trajOne.end())
                .lineToLinearHeading(new Pose2d(36, -36, Math.toRadians(270)))
                .build();

        middle_trajOne = robot.trajectorySequenceBuilder(startPos)
                .lineToLinearHeading(new Pose2d(14, -31.75, Math.toRadians(90)))
                .forward(20)
                .back(14)
                .build();

        middle_trajTwo = robot.trajectorySequenceBuilder(middle_trajOne.end())
                .lineToLinearHeading(new Pose2d(14, -45, Math.toRadians(90)))
                .build();
        right_trajOne = robot.trajectorySequenceBuilder(startPos)
                .lineTo(new Vector2d(25, -61))
                .lineTo(new Vector2d(25, -40))
                .strafeRight(12)
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

    Pose2d startPos = new Pose2d(14, -60, Math.toRadians(90));
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
            case FINISH:
                if (!robot.isBusy()) {
                    stop();
                }
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
                    int leftAmt = 1;
                    int rightAmt = 1;
                    if (currentState == State.LEFT) rightAmt = 20;
                    if (currentState == State.RIGHT) leftAmt = 25;
                    drop_trajOne = robot.trajectorySequenceBuilder(robot.getPoseEstimate())
                            .splineTo(new Vector2d(40, -36), Math.toRadians(180))
                            .strafeRight(rightAmt)
                            .strafeLeft(leftAmt)
                            .turn(Math.toRadians(5))
                            .back(10)
                            .build();
                    currentStep = Step.DROP;
                    robot.followTrajectorySequenceAsync(drop_trajOne);
                }
                break;
            case ONE:
                if (!robot.isBusy()) {
                    left_trajTwo = robot.trajectorySequenceBuilder(robot.getPoseEstimate())
                            .lineToLinearHeading(new Pose2d(36, -36, Math.toRadians(270)))
                            .build();

                    middle_trajTwo = robot.trajectorySequenceBuilder(robot.getPoseEstimate())
                            .lineToLinearHeading(new Pose2d(14, -45, Math.toRadians(90)))
                            .build();
                    right_trajTwo = robot.trajectorySequenceBuilder(robot.getPoseEstimate())
                            .lineTo(new Vector2d(25, -55))
                            .lineToLinearHeading(new Pose2d(30, -36, Math.toRadians(270)))
                            .build();
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
        intake.setPower(INTAKE_POWER, 1);
        roller.spinForward();
        sleep(300);
        intake.stop();
        roller.stop();
    }
}
