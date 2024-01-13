package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class CameraScanSubsystem implements Subsystem {

    public int wanted = 1;
    public AprilTagDetection detection;
    @Override
    public void init(SampleMecanumDrive map) {
        AprilTagProcessor processor = new AprilTagProcessor.Builder()
                .build();
        VisionPortal portal = new VisionPortal.Builder()
                .addProcessor(processor)
                .setCamera(map.cameraLeft)
                .build();
    }

    List<AprilTagDetection> detections;

    public AprilTagDetection getDetection() { return this.detection; }

    @Override
    public void run(Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry) {
        for(AprilTagDetection detection : detections) {
            if (detection.id == this.wanted) {
                telemetry.addData("Found AprilTag with ID", this.wanted);
                this.detection = detection;
            }
        }
    }
}
