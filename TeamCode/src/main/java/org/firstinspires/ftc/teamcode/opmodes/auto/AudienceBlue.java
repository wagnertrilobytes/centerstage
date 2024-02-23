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
@Autonomous(name = "Audience Blurfytgue", group="Backstage")
public class AudienceBlue extends ColorVisionAutoBase {
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
        Storage.currentPose = robot.getPoseEstimate();

//        left_trajOne = robot.trajectorySequenceBuilder(startPos)
//                .splineTo(new Vector2d(32, 30), Math.toRadians(180))
//                .splineTo(new Vector2d(14, 32), Math.toRadians(180))
//                .build();

        right_trajOne =robot.trajectorySequenceBuilder(startPos)
                .lineTo(new Vector2d(-46.26, 62.00))
                .lineTo(new Vector2d(-46.52, 35.52))
                .build();


        right_trajTwo = robot.trajectorySequenceBuilder(right_trajOne.end())
                .lineTo(new Vector2d(-46.26, 45))
                .lineTo(new Vector2d(-56.26, 41.58))
//                .lineToConstantHeading(ENDPOS)
                .build();
        middle_trajOne  = robot.trajectorySequenceBuilder(startPos)
                .lineToConstantHeading(new Vector2d(-35.95, 34.18))
                .build();


        middle_trajTwo = robot.trajectorySequenceBuilder(middle_trajOne.end())
                .lineToConstantHeading(new Vector2d(-35.95, 40.18))
//                .splineTo(ENDPOS, Math.toRadians(0))
                .build();

        left_trajOne = robot.trajectorySequenceBuilder(startPos)
                .splineTo(new Vector2d(-40.13, 45.36), Math.toRadians(270.00))
                .splineTo(new Vector2d(-34.18, 34.02), Math.toRadians(-8.77))
                .build();
        left_trajTwo = robot.trajectorySequenceBuilder(left_trajOne.end())
                .lineTo(new Vector2d(-54.33, 37.49))
//                .lineToLinearHeading(ENDPOSE)
                .build();
    }

    Pose2d startPos = new Pose2d(-37, 66, Math.toRadians(270.00));
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
                    sleep(2250);
                    slides.stop();
                    bucketCR.setSlideArmPower(-bArmSpeed);
                    sleep(150);
                    bucketCR.setSlideArmPower(0);
                    bucketCR.setWheelPower(-1);
                    bucketCR.setBucketArmPower(-0.3);
                    sleep(1500);
                    bucketCR.stop();
                }
                break;
            case TWO:
                if (!robot.isBusy()) {
                    drop_trajOne = robot.trajectorySequenceBuilder(robot.getPoseEstimate())
                            .lineToLinearHeading(new Pose2d(37, 36, Math.toRadians(180)))
                            .strafeLeft(10)
                            .back(20)
                            .build();
                    currentStep = Step.DROP;
                    robot.followTrajectorySequenceAsync(drop_trajOne);
                }
                break;
            case ONE:
                if (!robot.isBusy()) {
                    right_trajTwo = robot.trajectorySequenceBuilder(robot.getPoseEstimate())
                            .lineTo(new Vector2d(-46.26, 45))
                            .lineTo(new Vector2d(-56.26, 41.58))
//                .lineToConstantHeading(ENDPOS)
                            .build();


                    middle_trajTwo = robot.trajectorySequenceBuilder(robot.getPoseEstimate())
                            .lineToConstantHeading(new Vector2d(-35.95, 40.18))
//                .splineTo(ENDPOS, Math.toRadians(0))
                            .build();
                    left_trajTwo = robot.trajectorySequenceBuilder(robot.getPoseEstimate())
                            .lineTo(new Vector2d(-54.33, 37.49))
//                .lineToLinearHeading(ENDPOSE)
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
        sleep(300);
        intake.stop();
    }
}
