package org.firstinspires.ftc.teamcode.opmodes.test;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name="April tag test!!!", group = "Tests")
@Config
public class AprilTagScanningTest extends LinearOpMode {
    public static int WANTED_ID = 1;
    public static double INCHES_AWAY = 8.5;
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
            List<AprilTagDetection> currentDetections = aprilTags.getDetections();
            double calculatedDist = 0;
            AprilTagDetection detection = null;
            for (AprilTagDetection currDet : currentDetections) {
                if(currDet.id != WANTED_ID) {

                } else {
                    detection = currDet;
                }
                // Look to see if we have a tag.
            }

            String msg = "Thumbs up";
            if (currentDetections.size() == 0) msg = "Not currently seeing any tag. Using last known position.";
            telemetry.addData("Status", msg);

            if (detection != null) {
                calculatedDist = detection.ftcPose.range - INCHES_AWAY;
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
                telemetry.addData("Gamepad1 A", "Turn & go to that tag (move forward " + calculatedDist + "in)");
                telemetry.addData("Gamepad1 B", "Face tag [BEARING] (" + detection.ftcPose.bearing + "deg)");
                telemetry.addData("Gamepad1 X", "RUN AN EXAMPLE TEST");
                telemetry.update();
                TrajectorySequence newTraj = robot.trajectorySequenceBuilder(robot.getPoseEstimate())
                        .turn(Math.toRadians(detection.ftcPose.bearing))
                        .forward(calculatedDist)
                        .turn(Math.toRadians(-detection.ftcPose.bearing))
                        .build();

                if (gamepad1.a) {
                    robot.followTrajectorySequence(newTraj);
                }
                if (gamepad1.b) {
                    TrajectorySequence trajsec = robot.trajectorySequenceBuilder(robot.getPoseEstimate())
                            .turn(Math.toRadians(detection.ftcPose.bearing))
                            .build();
                    robot.followTrajectorySequence(trajsec);
                }
            }
            if (gamepad1.x) {
                double setMotorTime = 2; // What time we set the motor power
                double setMotorWait = 0.35; // How long we wait until we turn off the motor
                Pose2d startPos =new Pose2d(14, 60, Math.toRadians(270));

                robot.setPoseEstimate(startPos);
                // This is a test!
                TrajectorySequence left_trajOne =robot.trajectorySequenceBuilder(startPos)
                        .lineTo(new Vector2d(25, 61))
                        .lineTo(new Vector2d(25, 40))
                        .addTemporalMarker(setMotorTime, () -> {
                            robot.intake.setPower(0.75);
                        })
                        .addTemporalMarker(setMotorTime + setMotorWait, () -> {
                            robot.intake.setPower(0);
                        })
                        .lineToLinearHeading(new Pose2d(36, 36, Math.toRadians(0)))
                        .build();


                robot.followTrajectorySequence(left_trajOne);
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
                calculatedDist = apr.ftcPose.range - INCHES_AWAY;
                double doSUP = 2.5;
                double doSUPW = 2.0;
                double doSUPWE = 1.75;
                TrajectorySequence newTraj = robot.trajectorySequenceBuilder(robot.getPoseEstimate())
                        .turn(Math.toRadians(apr.ftcPose.bearing))
                        .forward(calculatedDist)
                        .turn(-Math.toRadians(apr.ftcPose.bearing))
                        .turn(Math.toRadians(180))
                        .back(4)
                        .addTemporalMarker(doSUP, () -> {
                            robot.slideLeft.setPower(-0.75);
                            robot.slideRight.setPower(0.75);
                            robot.clawLeft.turnToAngle(270);
                            robot.clawRight.turnToAngle(270);
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
                            robot.clawLeft.turnToAngle(270);
                            robot.clawRight.turnToAngle(270);
                        })
                        .addTemporalMarker(doSUP+ doSUPW+doSUPWE+doSUPW, () -> {
                            robot.slideLeft.setPower(0);
                            robot.slideRight.setPower(0);
                            robot.clawLeft.turnToAngle(0);
                            robot.clawRight.turnToAngle(0);
                        })
                        .build();
                robot.followTrajectorySequence(newTraj);
            }
        }
    }

}
