package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.vision.ColorVisionAutoBase;
import org.firstinspires.ftc.teamcode.vision.ColourMassDetectionProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Scalar;

import java.lang.annotation.Target;
import java.util.List;
import java.util.Timer;

@Disabled()
@Autonomous(name = "Backstage Blue DO NOT ENABLE OR ELSE!!!!!!!", group="Backstage")
public class BackstageBlue extends ColorVisionAutoBase {
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

//        left_trajOne = robot.trajectorySequenceBuilder(startPos)
//                .splineTo(new Vector2d(32, 30), Math.toRadians(180))
//                .splineTo(new Vector2d(14, 32), Math.toRadians(180))
//                .build();

        left_trajOne =robot.trajectorySequenceBuilder(startPos)
                .lineTo(new Vector2d(25, 61))
                .lineTo(new Vector2d(25, 40))
                .build();

        left_trajTwo = robot.trajectorySequenceBuilder(left_trajOne.end())
                .lineToLinearHeading(new Pose2d(36, 36, Math.toRadians(9)))
                .build();

        middle_trajOne = robot.trajectorySequenceBuilder(startPos)
                .lineToConstantHeading(new Vector2d(14, 32.86))
                .build();

        middle_trajTwo = robot.trajectorySequenceBuilder(middle_trajOne.end())
                .lineToConstantHeading(new Vector2d(11.89, 44.09))
                .lineTo(new Vector2d(36.00, 36.00))
                .build();

        right_trajOne = robot.trajectorySequenceBuilder(startPos)
                .lineTo(new Vector2d(25, 61))
                .lineTo(new Vector2d(25, 40))
                .lineToLinearHeading(new Pose2d(10, 36, Math.toRadians(180)))
                .build();
        right_trajTwo = robot.trajectorySequenceBuilder(right_trajOne.end())
                .lineToLinearHeading(new Pose2d(36, 36, Math.toRadians(9)))
                .build();

        drop_trajOne = robot.trajectorySequenceBuilder(robot.getPoseEstimate())
                .splineTo(new Vector2d(35, 36), Math.toRadians(0))
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
        aprilTags = new AprilTagProcessor.Builder().build();
        vp = new VisionPortal.Builder()
                .setCamera(robot.cameraLeft)
                .addProcessor(aprilTags)
                .build();
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
    public void onStartedColor(ColourMassDetectionProcessor.Prop detectedProp) {;
        // now we can use recordedPropPosition in our auto code to modify where we place the purple and yellow pixels
        switch (currentStep) {
            case FINISH:
                if (!robot.isBusy()) {
//                    stop();
                }
                break;
            case DROP:
                if (!robot.isBusy()) {
                    currentStep = Step.FINISH;
                    // Make sure camera is streaming before we try to set the exposure controls

                    AprilTagDetection apr = null;
                    do {
                        List<AprilTagDetection> currentDetectionsGT = aprilTags.getDetections();
                        for (AprilTagDetection currDet : currentDetectionsGT) {
                            telemetry.addData("FOUND", currDet.id);
                            if(currDet.id != WANTED_ID) {

                            } else {
                                apr = currDet;
                            }
                            // Look to see if we have a tag.
                        }
                    }
                    while(apr == null);
                    double calculatedDist = apr.ftcPose.range - INCHES_AWAY;
                    double doSUP = 3.25;
                    double doSUPW = 2.0;
                    double doSUPWE = 2.2;
                    double lastTimer = 0.75;
                    TrajectorySequence newTraj = robot.trajectorySequenceBuilder(robot.getPoseEstimate())
                            .turn(Math.toRadians(apr.ftcPose.bearing))
                            .forward(calculatedDist)
                            .turn(-Math.toRadians(apr.ftcPose.bearing))
                            .turn(Math.toRadians(180))
                            .back(6)
                            .addTemporalMarker(doSUP, () -> {
                                robot.slideLeft.setPower(-0.75);
                                robot.slideRight.setPower(0.75);
                                robot.clawLeft.turnToAngle(robot.clawLeft.max - 1);
                                robot.clawRight.turnToAngle(robot.clawLeft.max - 1);
                            })
                            .addTemporalMarker(doSUP+ doSUPW, () -> {
                                robot.slideLeft.setPower(0);
                                robot.slideRight.setPower(0);
                                robot.clawLeft.turnToAngle(0);
                                robot.clawRight.turnToAngle(0);
                            })
                            .addTemporalMarker(doSUP+doSUPW+doSUPWE, () -> {
                                robot.slideLeft.setPower(0.75);
                                robot.slideRight.setPower(-0.75);
                                robot.clawLeft.turnToAngle(robot.clawLeft.max - 1);
                                robot.clawRight.turnToAngle(robot.clawLeft.max - 1);
                            })
                            .addTemporalMarker(doSUP+ doSUPW+doSUPWE+doSUPW+lastTimer, () -> {
                                robot.slideLeft.setPower(0);
                                robot.slideRight.setPower(0);
                                robot.clawLeft.turnToAngle(6);
                                robot.clawRight.turnToAngle(6);
                            })
                            .build();
                    robot.followTrajectorySequenceAsync(newTraj);
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
        robot.intake.setPower(-INTAKE_POWER);
        sleep(300);
        robot.intake.setPower(0);
    }
}
