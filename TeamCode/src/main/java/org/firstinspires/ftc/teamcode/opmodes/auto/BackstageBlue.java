package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.checkerframework.checker.units.qual.C;
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

    static double bArmSpeed = 0.75;
    static double bBucSpeed = 0.75;
    static double slideSpeed = 0.75;
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

//        left_trajOne = robot.trajectorySequenceBuilder(startPos)
//                .splineTo(new Vector2d(32, 30), Math.toRadians(180))
//                .splineTo(new Vector2d(14, 32), Math.toRadians(180))
//                .build();

        left_trajOne =robot.trajectorySequenceBuilder(startPos)
                .lineTo(new Vector2d(25, 61))
                .lineTo(new Vector2d(23, 39))
                .build();

        left_trajTwo = robot.trajectorySequenceBuilder(left_trajOne.end())
                .lineTo(new Vector2d(24, 60))
                .build();

        middle_trajOne = robot.trajectorySequenceBuilder(startPos)
                .lineToConstantHeading(new Vector2d(14, 34))
                .build();

        middle_trajTwo = robot.trajectorySequenceBuilder(middle_trajOne.end())
                .lineToConstantHeading(new Vector2d(14, 44.09))
                .lineTo(midbefDropV)
                .build();

        right_trajOne = robot.trajectorySequenceBuilder(startPos)
                .lineTo(new Vector2d(25, 61))
                .lineTo(new Vector2d(25, 40))
                .build();

        right_trajTwo = robot.trajectorySequenceBuilder(right_trajOne.end())
                .lineTo(new Vector2d(24, 60))
                .build();

        drop_trajOne = robot.trajectorySequenceBuilder(midbefDrop)
                .splineTo(new Vector2d(37, 36), Math.toRadians(180))
                .back(15)
                .build();
    }

    Pose2d startPos = new Pose2d(14, 60, Math.toRadians(270));
    Pose2d midbefDrop = new Pose2d(14, 44, Math.toRadians(180));
    Vector2d midbefDropV = new Vector2d(14, 44);
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
                    sleep(750);
                    slides.stop();
                    sleep(250);
                    bucketCR.setSlideArmPower(bArmSpeed);
                    bucketCR.setBucketArmPower(bBucSpeed);
                    sleep(750);
                    bucketCR.stop();
                    sleep(350);
                    slides.setPower(-slideSpeed);
                    sleep(750);
                    slides.stop();
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
        intake.setPower(INTAKE_POWER, 1);
        roller.spinForward();
        sleep(300);
        intake.stop();
        roller.stop();
    }
}
