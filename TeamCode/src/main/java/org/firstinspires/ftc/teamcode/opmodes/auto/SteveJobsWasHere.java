package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.helpers.Storage;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.vision.ColourMassDetectionProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.opencv.core.Scalar;

import java.util.function.DoubleSupplier;

@Autonomous(name = "Hooligan Activity", group = "Its so sad that steve jobs died of ligma")
public class SteveJobsWasHere extends LinearOpMode {
    private VisionPortal visionPortal;
    public ColourMassDetectionProcessor colourMassDetectionProcessor;
    public Scalar lower = new Scalar(150, 100, 100); // the lower hsv threshold for your detection
    public Scalar upper = new Scalar(180, 255, 255); // the upper hsv threshold for your detection
    public DoubleSupplier minArea = () -> 100; // the minimum area for the detection to consider for your prop
    public DoubleSupplier left = () -> 213;
    public DoubleSupplier right = () -> 426;
    public String name = "None";
    public int WANTED_ID = 1;
    public SampleMecanumDrive robot;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        this.robot = new SampleMecanumDrive(hardwareMap);
        Pose2d startPos = new Pose2d(14, 60, Math.toRadians(270));

        robot.setPoseEstimate(startPos);

        TrajectorySequence left_trajOne =robot.trajectorySequenceBuilder(startPos)
                .lineTo(new Vector2d(25, 61))
                .lineTo(new Vector2d(23, 39))
                .build();

        TrajectorySequence left_trajTwo = robot.trajectorySequenceBuilder(left_trajOne.end())
                .lineTo(new Vector2d(25, 61))
                .build();

        TrajectorySequence middle_trajOne = robot.trajectorySequenceBuilder(startPos)
                .lineToConstantHeading(new Vector2d(14, 33))
                .build();

        TrajectorySequence middle_trajTwo = robot.trajectorySequenceBuilder(middle_trajOne.end())
                .lineToConstantHeading(new Vector2d(11.89, 44.09))
                .build();

        TrajectorySequence right_trajOne = robot.trajectorySequenceBuilder(startPos)
                .lineTo(new Vector2d(25, 61))
                .lineTo(new Vector2d(25, 40))
                .lineToLinearHeading(new Pose2d(10, 36, Math.toRadians(180)))
                .build();

        TrajectorySequence drop_trajOne = robot.trajectorySequenceBuilder(robot.getPoseEstimate())
                .splineTo(new Vector2d(35, 36), Math.toRadians(180))
                .back(7)
                .back(7)
                .back(7)
                .build();

        colourMassDetectionProcessor = new ColourMassDetectionProcessor(this.lower, this.upper, this.minArea, this.left, this.right);
        visionPortal = new VisionPortal.Builder()
                .setCamera(robot.cameraLeft) // the camera on your robot is named "Webcam 1" by default
                .addProcessor(colourMassDetectionProcessor)
                .build();
        while(opModeInInit()) {
            telemetry.addData("Currently Recorded Position", colourMassDetectionProcessor.getRecordedPropPosition());
            telemetry.addData("Camera State", visionPortal.getCameraState());
            telemetry.addData("Currently Detected Mass Center", "x: " + colourMassDetectionProcessor.getLargestContourX() + ", y: " + colourMassDetectionProcessor.getLargestContourY());
            telemetry.addData("Currently Detected Mass Area", colourMassDetectionProcessor.getLargestContourArea());
            telemetry.update();
        }
        waitForStart();
        if (visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
            visionPortal.stopLiveView();
            visionPortal.stopStreaming();
        }

        ColourMassDetectionProcessor.PropPositions recordedPropPositionL = colourMassDetectionProcessor.getRecordedPropPosition();

        if (recordedPropPositionL == ColourMassDetectionProcessor.PropPositions.UNFOUND) recordedPropPositionL = ColourMassDetectionProcessor.PropPositions.MIDDLE;

        ColourMassDetectionProcessor.Prop detectedProp = new ColourMassDetectionProcessor.Prop(recordedPropPositionL, this.name);

        robot.clawLeft.turnToAngle(7);
        robot.clawRight.turnToAngle(7);
        if (detectedProp.getPosition() == ColourMassDetectionProcessor.PropPositions.LEFT) {
            WANTED_ID = 1;
            robot.followTrajectorySequence(left_trajOne);
        } else if(detectedProp.getPosition() == ColourMassDetectionProcessor.PropPositions.RIGHT) {
            WANTED_ID = 3;
            robot.followTrajectorySequence(right_trajOne);
        } else if(detectedProp.getPosition() == ColourMassDetectionProcessor.PropPositions.MIDDLE) {
            WANTED_ID = 2;
            robot.followTrajectorySequence(middle_trajOne);
        }

        while(opModeIsActive() && !isStopRequested()) {
            doIntakeSpin();
            if (detectedProp.getPosition() == ColourMassDetectionProcessor.PropPositions.MIDDLE) robot.followTrajectorySequence(middle_trajTwo);
            if (detectedProp.getPosition() == ColourMassDetectionProcessor.PropPositions.LEFT) robot.followTrajectorySequence(left_trajTwo);

            robot.clawLeft.turnToAngle(13);
            robot.clawRight.turnToAngle(13);

            robot.followTrajectorySequenceAsync(drop_trajOne);

            robot.slideLeft.setPower(-0.75);
            robot.slideRight.setPower(0.75);
            sleep(1000);
            robot.slideLeft.setPower(0);
            robot.slideRight.setPower(0);
            sleep(250);
            robot.clawLeft.turnToAngle(robot.clawLeft.max - 15);
            robot.clawRight.turnToAngle(robot.clawRight.max - 15);
            sleep(250);

            Pose2d poseEstimate = this.robot.getPoseEstimate();
            Storage.currentPose = poseEstimate;
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();
        }
    }
    public void doIntakeSpin() {
        robot.intake.setPower(-0.5);
        sleep(450);
        robot.intake.setPower(0);
    }
}
