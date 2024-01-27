package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.helpers.Storage;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.vision.ColourMassDetectionProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.opencv.core.Scalar;

import java.util.function.DoubleSupplier;

@Disabled
@Autonomous(name = "New backstage blu8 oikeger")
public class NewBackstageBlue extends LinearOpMode {
    TrajectorySequence right_trajOne;
    TrajectorySequence right_trajTwo;
    TrajectorySequence left_trajOne;
    TrajectorySequence left_trajTwo;

    TrajectorySequence middle_trajOne;
    TrajectorySequence middle_trajTwo;

    TrajectorySequence drop_trajOne;
    TrajectorySequence drop_trajTwo;
    SampleMecanumDrive robot;
    @Override
    public void runOpMode() throws InterruptedException {
        Scalar lower = new Scalar(40, 100, 100); // the lower hsv threshold for your detection
        Scalar upper = new Scalar(180, 255, 255); // the upper hsv threshold for your detection
        DoubleSupplier minArea = () -> 2000; // the minimum area for the detection to consider for your prop
        DoubleSupplier left = () -> 213;
        DoubleSupplier right = () -> 426;
        Pose2d startPos = new Pose2d(14, 60, Math.toRadians(270));
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot = new SampleMecanumDrive(hardwareMap);
        ColourMassDetectionProcessor colourMassDetectionProcessor = new ColourMassDetectionProcessor(lower, upper, minArea, left, right);
        VisionPortal visionPortal = new VisionPortal.Builder()
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
        robot.setPoseEstimate(startPos);
        waitForStart();
        if (isStopRequested()) return;
        if (visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
            visionPortal.stopLiveView();
            visionPortal.stopStreaming();
        }

        ColourMassDetectionProcessor.PropPositions recordedPropPositionL = colourMassDetectionProcessor.getRecordedPropPosition();

        // now we can use recordedPropPosition to determine where the prop is! if we never saw a prop, your recorded position will be UNFOUND.
        // if it is UNFOUND, you can manually set it to any of the other positions to guess
        if (recordedPropPositionL == ColourMassDetectionProcessor.PropPositions.UNFOUND) recordedPropPositionL = ColourMassDetectionProcessor.PropPositions.MIDDLE;

        // ON STARTED

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
                .lineToConstantHeading(new Vector2d(11.89, 44.09))
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
        colourMassDetectionProcessor.close();
        visionPortal.close();

        robot.clawLeft.turnToAngle(18);
        robot.clawRight.turnToAngle(18);
        if (recordedPropPositionL == ColourMassDetectionProcessor.PropPositions.LEFT) robot.followTrajectorySequence(left_trajOne);
        if (recordedPropPositionL == ColourMassDetectionProcessor.PropPositions.MIDDLE) robot.followTrajectorySequence(middle_trajOne);
        if (recordedPropPositionL == ColourMassDetectionProcessor.PropPositions.RIGHT) robot.followTrajectorySequence(right_trajOne);
        // now we can use recordedPropPosition in our auto code to modify where we place the purple and yellow pixels
//        doIntakeSpin(-0.4);
        if (recordedPropPositionL == ColourMassDetectionProcessor.PropPositions.MIDDLE) robot.followTrajectorySequence(middle_trajTwo);
        if (recordedPropPositionL == ColourMassDetectionProcessor.PropPositions.LEFT) robot.followTrajectorySequence(left_trajTwo);
        robot.clawLeft.turnToAngle(18);
        robot.clawRight.turnToAngle(18);
        robot.followTrajectorySequence(drop_trajOne);

        robot.slideLeft.setPower(-0.75);
        robot.slideRight.setPower(0.75);
        sleep(1000);
        robot.slideLeft.setPower(0);
        robot.slideRight.setPower(0);
        sleep(250);
        robot.clawLeft.turnToAngle(robot.clawLeft.max - 15);
        robot.clawRight.turnToAngle(robot.clawRight.max - 15);
        sleep(250);
    }

    public void doIntakeSpin(double intakePower) {
        robot.intake.setPower(-intakePower);
        sleep(450);
        robot.intake.setPower(0);
    }
}
