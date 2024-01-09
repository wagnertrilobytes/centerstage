package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Autonomous(name="April tag test!!!", group = "Tests")
public class AprilTagScanningTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        SampleMecanumDrive robot = new SampleMecanumDrive(hardwareMap);
        AprilTagProcessor aprilTags = new AprilTagProcessor.Builder().build();
        VisionPortal vp = new VisionPortal.Builder()
                .setCamera(robot.cameraLeft)
                .addProcessor(aprilTags)
                .build();
        // Make sure camera is streaming before we try to set the exposure controls
        if (vp.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (vp.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }
        waitForStart();
        while (!isStopRequested() && opModeIsActive()) {
            AprilTagDetection currTag = null;
            List<AprilTagDetection> currentDetections = aprilTags.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                // Look to see if we have size info on this tag.
                telemetry.addData("Tag ID", detection.id);
                telemetry.addData("x", detection.ftcPose.x);
                telemetry.addData("y", detection.ftcPose.y);
                telemetry.addData("z", detection.ftcPose.z);
                telemetry.addData("yaw", detection.ftcPose.yaw);
                telemetry.addData("pitch", detection.ftcPose.pitch);
                telemetry.addData("roll", detection.ftcPose.roll);
                telemetry.addData("bearing", detection.ftcPose.bearing);
                telemetry.addData("range", detection.ftcPose.range);
                telemetry.addData("elevation", detection.ftcPose.elevation);
            }
            double calculatedDist = currTag.ftcPose.range - 5;
            telemetry.addData("Gamepad1 A", "Go to that tag (move forward " + calculatedDist +"in)");
            telemetry.addData("Gamepad1 B", "Face tag [BEARING] (" + currTag.ftcPose.bearing+"deg)");
            telemetry.addData("Gamepad1 X", "Face tag [YAW] (" + currTag.ftcPose.yaw+"deg)");
            telemetry.addData("Gamepad1 Y", "Face tag [PITCH] (" + currTag.ftcPose.pitch+"deg)");
            telemetry.update();

            if (gamepad1.a) {
                TrajectorySequence trajsec = robot.trajectorySequenceBuilder(robot.getPoseEstimate())
                        .forward(calculatedDist)
                        .build();
                robot.followTrajectorySequenceAsync(trajsec);
            }
            if (gamepad1.b) {
                TrajectorySequence trajsec = robot.trajectorySequenceBuilder(robot.getPoseEstimate())
                        .turn(Math.toRadians(currTag.ftcPose.bearing))
                        .build();
                robot.followTrajectorySequenceAsync(trajsec);
            }
            if (gamepad1.b) {
                TrajectorySequence trajsec = robot.trajectorySequenceBuilder(robot.getPoseEstimate())
                        .turn(Math.toRadians(currTag.ftcPose.yaw))
                        .build();
                robot.followTrajectorySequenceAsync(trajsec);
            }
            if (gamepad1.b) {
                TrajectorySequence trajsec = robot.trajectorySequenceBuilder(robot.getPoseEstimate())
                        .turn(Math.toRadians(currTag.ftcPose.pitch))
                        .build();
                robot.followTrajectorySequenceAsync(trajsec);
            }
        }
    }

}
