package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.vision.ColorVisionAutoBase;
import org.firstinspires.ftc.teamcode.vision.ColourMassDetectionProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Scalar;

import java.util.List;

@Autonomous(name = "Backstage Blue", group="Backstage")
public class BackstageBlueToo extends ColorVisionAutoBase {
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
                .lineTo(new Vector2d(25, 61))
                .build();

        middle_trajOne = robot.trajectorySequenceBuilder(startPos)
                .lineToConstantHeading(new Vector2d(14, 33))
                .build();

        middle_trajTwo = robot.trajectorySequenceBuilder(middle_trajOne.end())
                .lineToConstantHeading(new Vector2d(14, 44.09))
                .build();

        right_trajOne = robot.trajectorySequenceBuilder(startPos)
                .lineTo(new Vector2d(25, 61))
                .lineTo(new Vector2d(25, 40))
                .lineToLinearHeading(new Pose2d(10, 36, Math.toRadians(180)))
                .build();

        drop_trajOne = robot.trajectorySequenceBuilder(robot.getPoseEstimate())
                .splineTo(new Vector2d(35, 36), Math.toRadians(180))
                .back(7)
                .back(7)
                .back(7)
                .build();

    }

    Pose2d startPos = new Pose2d(14, 60, Math.toRadians(270));
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
        robot.clawLeft.turnToAngle(2);
        robot.clawRight.turnToAngle(2);
        currentStep = Step.ONE;
        if (detectedProp.getPosition() == ColourMassDetectionProcessor.PropPositions.LEFT) {
            currentState = State.LEFT;
            WANTED_ID = 1;
            robot.followTrajectorySequenceAsync(left_trajOne);
        } else if(detectedProp.getPosition() == ColourMassDetectionProcessor.PropPositions.RIGHT) {
            currentState = State.RIGHT;
            WANTED_ID = 3;
            robot.followTrajectorySequenceAsync(right_trajOne);
        } else if(detectedProp.getPosition() == ColourMassDetectionProcessor.PropPositions.MIDDLE) {
            currentState = State.MIDDLE;
            WANTED_ID = 2;
            robot.followTrajectorySequenceAsync(middle_trajOne);
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
                    sleep(1000);
                    robot.slideLeft.setPower(0);
                    robot.slideRight.setPower(0);
                    sleep(250);
                    robot.clawLeft.turnToAngle(robot.clawLeft.max - 15);
                    robot.clawRight.turnToAngle(robot.clawRight.max - 15);
                    sleep(500);
                    robot.clawLeft.turnToAngle(0);
                    robot.clawRight.turnToAngle(0);
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
