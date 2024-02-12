package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Slides;
import org.firstinspires.ftc.teamcode.subsystems.SpicyBucket;
import org.firstinspires.ftc.teamcode.vision.ColorVisionAutoBase;
import org.firstinspires.ftc.teamcode.vision.ColourMassDetectionProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Scalar;

@Autonomous(name = "Backstage Red Latest Test", group="Backstage", preselectTeleOp = "Centerstage: Teleop PizzaBox Lives On")
public class BackstageRedSync extends ColorVisionAutoBase {
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
    Intake intake = new Intake();
    Slides slides = new Slides();
    SpicyBucket spicyBucket = new SpicyBucket();

    @Override
    public void setup() {
        intake.init(hardwareMap);
        slides.init(hardwareMap);
        spicyBucket.init(hardwareMap);

        this.lower = new Scalar(0, 100, 100); // the lower hsv threshold for your detection
        this.upper = new Scalar(255, 255, 255); // the upper hsv threshold for your detection
        this.minArea = () -> 2000; // the minimum area for the detection to consider for your prop
        this.left = () -> 213;
        this.right = () -> 426;
        this.name = "Red";
        robot.setPoseEstimate(startPos);

        left_trajOne = robot.trajectorySequenceBuilder(startPos)
                .splineTo(new Vector2d(32, -30), Math.toRadians(180))
                .splineTo(new Vector2d(10, -32), Math.toRadians(180))
                .build();

        left_trajTwo = robot.trajectorySequenceBuilder(left_trajOne.end())
                .lineToLinearHeading(new Pose2d(36, -36, Math.toRadians(270)))
                .build();

        middle_trajOne = robot.trajectorySequenceBuilder(startPos)
                .lineToLinearHeading(new Pose2d(14, -31.75, Math.toRadians(90)))
                .build();

        middle_trajTwo = robot.trajectorySequenceBuilder(middle_trajOne.end())
                .lineToLinearHeading(new Pose2d(14, -45, Math.toRadians(90)))
                .build();
        right_trajOne = robot.trajectorySequenceBuilder(startPos)
                .lineTo(new Vector2d(25, -61))
                .lineTo(new Vector2d(25, -40))
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
        telemetry.addData("Step", currentStep);
        telemetry.addData("State", currentState);
        currentStep = Step.TWO;
        doIntakeSpin();
        if (currentState == State.MIDDLE) robot.followTrajectorySequence(middle_trajTwo);
        if (currentState == State.LEFT) robot.followTrajectorySequence(left_trajTwo);
        if (currentState == State.RIGHT) robot.followTrajectorySequence(right_trajTwo);
        currentStep = Step.DROP;
        robot.followTrajectorySequence(drop_trajOne);
        currentStep = Step.FINISH;
        slides.setPower(0.75);
        sleep(1250);
        slides.stop();
        sleep(250);
        spicyBucket.setSlideArmAngle(spicyBucket.maxArmAngle() - 15);
        sleep(500);
        spicyBucket.dropOnePixel(this);
        sleep(1500);
        spicyBucket.setSlideArmAngle(spicyBucket.minArmAngle());
        sleep(250);
        slides.setPower(-0.75);
        sleep(1250);
        slides.stop();
    }

    public void doIntakeSpin() {
        intake.setPower(INTAKE_POWER, -1);
        sleep(450);
        intake.stop();
    }
}
