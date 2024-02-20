package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Slides;
import org.firstinspires.ftc.teamcode.subsystems.SpicyBucket;
import org.firstinspires.ftc.teamcode.subsystems.SpicyBucketCR;
import org.firstinspires.ftc.teamcode.vision.ColorVisionAutoBase;
import org.firstinspires.ftc.teamcode.vision.ColourMassDetectionProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Scalar;

@Autonomous(name = "Audience Blue", group="Backstage", preselectTeleOp = "Centerstage: Teleop PizzaBox Lives On")
public class AudienceBlueSync extends ColorVisionAutoBase {
    double WANTED_ID = 1;
    TrajectorySequence right_trajOne;
    TrajectorySequence right_trajTwo;
    TrajectorySequence left_trajOne;
    TrajectorySequence left_trajTwo;

    TrajectorySequence middle_trajOne;
    TrajectorySequence middle_trajTwo;
    Intake intake = new Intake();
    SpicyBucketCR spicyBucketCR = new SpicyBucketCR();

    @Override
    public void setup() {
        intake.init(hardwareMap);
        spicyBucketCR.init(hardwareMap);

        this.lower = new Scalar(0, 100, 100); // the lower hsv threshold for your detection
        this.upper = new Scalar(255, 255, 255); // the upper hsv threshold for your detection
        this.minArea = () -> 2000; // the minimum area for the detection to consider for your prop
        this.left = () -> 213;
        this.right = () -> 426;
        this.name = "Red";
        robot.setPoseEstimate(startPos);

        left_trajOne = robot.trajectorySequenceBuilder(startPos)
                .splineTo(new Vector2d(-40.13, 45.36), Math.toRadians(270.00))
                .splineTo(new Vector2d(-34.18, 34.02), Math.toRadians(-8.77))
                .build();

        left_trajTwo = robot.trajectorySequenceBuilder(left_trajOne.end())
                .lineTo(new Vector2d(-54.33, 37.49))
                .build();

        middle_trajOne = robot.trajectorySequenceBuilder(startPos)
                .lineToConstantHeading(new Vector2d(-35.95, 34.18))
                .build();

        middle_trajTwo = robot.trajectorySequenceBuilder(middle_trajOne.end())
                .lineToConstantHeading(new Vector2d(-35.95, 40.18))
                .build();
        right_trajOne = robot.trajectorySequenceBuilder(startPos)
                .lineTo(new Vector2d(-46.26, 62.00))
                .lineTo(new Vector2d(-46.52, 35.52))
                .build();
        right_trajTwo = robot.trajectorySequenceBuilder(right_trajOne.end())
                .lineTo(new Vector2d(-46.26, 45))
                .lineTo(new Vector2d(-56.26, 41.58))
                .build();

    }

    Pose2d startPos = new Pose2d(-37, 66, Math.toRadians(270.00));
    double INTAKE_POWER = 0.27;

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
        currentStep = Step.FINISH;
    }

    public void doIntakeSpin() {
        intake.setPower(INTAKE_POWER, -1);
        spicyBucketCR.dropOnePixel();
        sleep(550);
        intake.stop();
        spicyBucketCR.stop();
    }
}
