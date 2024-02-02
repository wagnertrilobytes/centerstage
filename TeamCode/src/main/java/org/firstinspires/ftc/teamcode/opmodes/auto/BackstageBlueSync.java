package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.vision.ColorVisionAutoBase;
import org.firstinspires.ftc.teamcode.vision.ColourMassDetectionProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Scalar;

@Autonomous(name = "Backstage Blue (NOT ASYNCHRONOUS", group="Backstage")
public class BackstageBlueSync extends ColorVisionAutoBase {
    double INCHES_AWAY = 7;
    double WANTED_ID = 1;
    TrajectorySequence right_trajOne;
    TrajectorySequence right_trajTwo;
    TrajectorySequence left_trajOne;
    TrajectorySequence left_trajTwo;

    TrajectorySequence middle_trajOne;
    TrajectorySequence middle_trajTwo;

    TrajectorySequence drop_trajOne;
    TrajectorySequence drop_trajTwo;
    VisionPortal vp;
    AprilTagProcessor aprilTags;

    @Override
    public void setup() {
        this.lower = new Scalar(40, 100, 100); // the lower hsv threshold for your detection
        this.upper = new Scalar(180, 255, 255); // the upper hsv threshold for your detection
        this.minArea = () -> 2000; // the minimum area for the detection to consider for your prop
        this.left = () -> 213;
        this.right = () -> 426;
        this.name = "Blue";
        robot.setPoseEstimate(startPos);

        left_trajOne =robot.trajectorySequenceBuilder(startPos)
                .lineTo(new Vector2d(25, 61))
                .lineTo(new Vector2d(23, 39))
                .build();

        left_trajTwo = robot.trajectorySequenceBuilder(left_trajOne.end())
                .lineTo(new Vector2d(24, 60))
                .build();

        middle_trajOne = robot.trajectorySequenceBuilder(startPos)
                .lineToConstantHeading(new Vector2d(14, 33))
                .back(3)
                .strafeLeft(5)
                .turn(Math.toRadians(-15))
                .forward(2.5)
                .build();

        middle_trajTwo = robot.trajectorySequenceBuilder(middle_trajOne.end())
                .lineToConstantHeading(new Vector2d(14, 44.09))
                .lineTo(midbefDropV)
                .build();

        right_trajOne = robot.trajectorySequenceBuilder(startPos)
                .lineTo(new Vector2d(25, 61))
                .lineTo(new Vector2d(25, 40))
                .lineToLinearHeading(new Pose2d(10, 36, Math.toRadians(180)))
                .build();

        right_trajTwo = robot.trajectorySequenceBuilder(right_trajOne.end())
                .lineTo(new Vector2d(24, 60))
                .build();

        drop_trajOne = robot.trajectorySequenceBuilder(midbefDrop)
                .splineTo(new Vector2d(35, 36), Math.toRadians(180))
                .back(15)
                .build();

    }

    Pose2d startPos = new Pose2d(14, 60, Math.toRadians(270));
    Pose2d midbefDrop = new Pose2d(14, 44, Math.toRadians(180));
    Vector2d midbefDropV = new Vector2d(14, 44);
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
        DROP,
        FINISH
    }

    State currentState = State.IDLE;
    Step currentStep = Step.ONE;

    @Override
    public void onStarted(ColourMassDetectionProcessor.Prop detectedProp) {
        robot.clawLeft.turnToAngle(robot.clawLeft.max - 2);
        robot.clawRight.turnToAngle(robot.clawRight.max - 2);
        currentStep = Step.ONE;
        if (detectedProp.getPosition() == ColourMassDetectionProcessor.PropPositions.LEFT) {
            currentState = State.LEFT;
            WANTED_ID = 1;
            robot.followTrajectorySequence(left_trajOne);
        } else if(detectedProp.getPosition() == ColourMassDetectionProcessor.PropPositions.RIGHT) {
            currentState = State.RIGHT;
            WANTED_ID = 3;
            robot.followTrajectorySequence(right_trajOne);
        } else if(detectedProp.getPosition() == ColourMassDetectionProcessor.PropPositions.MIDDLE) {
            currentState = State.MIDDLE;
            WANTED_ID = 2;
            robot.followTrajectorySequence(middle_trajOne);
        } else {
            currentState = State.DEFAULT;
        }
    }

    @Override
    public void onStartedColor(ColourMassDetectionProcessor.Prop detectedProp) {
        // now we can use recordedPropPosition in our auto code to modify where we place the purple and yellow pixels
        telemetry.addData("Step", currentStep);
        telemetry.addData("State", currentState);
        switch (currentStep) {
            case DROP:
                if (!robot.isBusy()) {
                    currentStep = Step.FINISH;
                    robot.slideLeft.setPower(-0.75);
                    robot.slideRight.setPower(0.75);
                    sleep(1250);
                    robot.slideLeft.setPower(0);
                    robot.slideRight.setPower(0);
                    sleep(250);
                    robot.clawLeft.turnToAngle(robot.clawLeft.max - 15);
                    robot.clawRight.turnToAngle(robot.clawRight.max - 15);
                    sleep(500);
                    robot.clawLeft.turnToAngle(0);
                    robot.clawRight.turnToAngle(0);
                    sleep(1500);
                    robot.clawLeft.turnToAngle(robot.clawLeft.max);
                    robot.clawRight.turnToAngle(robot.clawRight.max);
                    sleep(250);
                    robot.slideLeft.setPower(0.75);
                    robot.slideRight.setPower(-0.75);
                    sleep(1250);
                    robot.slideLeft.setPower(0);
                    robot.slideRight.setPower(0);
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
                        doIntakeSpin();
                    if (currentState == State.MIDDLE) robot.followTrajectorySequenceAsync(middle_trajTwo);
                    if (currentState == State.LEFT) robot.followTrajectorySequenceAsync(left_trajTwo);
                    if (currentState == State.RIGHT) robot.followTrajectorySequenceAsync(right_trajTwo);
                }
                break;
        }
    }

    public void doIntakeSpin() {
        robot.intake.setPower(-INTAKE_POWER);
        sleep(450);
        robot.intake.setPower(0);
    }
}
