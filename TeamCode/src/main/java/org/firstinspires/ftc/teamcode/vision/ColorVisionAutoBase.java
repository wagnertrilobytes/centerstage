package org.firstinspires.ftc.teamcode.vision;

import android.graphics.Color;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraControls;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.teamcode.helpers.Storage;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.opencv.core.Scalar;

import java.util.function.DoubleSupplier;

//@Disabled // remove this line to have this show up on your robot
//@Autonomous(name = "Backstage Red")
public class ColorVisionAutoBase extends LinearOpMode {
    private VisionPortal visionPortal;
    public ColourMassDetectionProcessor colourMassDetectionProcessor;
    public ColorBrightness brightness;
    public Scalar lower = new Scalar(150, 100, 100); // the lower hsv threshold for your detection
    public Scalar upper = new Scalar(180, 255, 255); // the upper hsv threshold for your detection
    public DoubleSupplier minArea = () -> 100; // the minimum area for the detection to consider for your prop
    public DoubleSupplier left = () -> 213;
    public DoubleSupplier right = () -> 426;
    public String name = "None";
    public SampleMecanumDrive robot;
    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        this.robot = new SampleMecanumDrive(hardwareMap);
        brightness = new ColorBrightness(hardwareMap, telemetry);
        setup();
        colourMassDetectionProcessor = new ColourMassDetectionProcessor(this.lower, this.upper, this.minArea, this.left, this.right);
        visionPortal = new VisionPortal.Builder()
                .setCamera(robot.cameraLeft) // the camera on your robot is named "Webcam 1" by default
                .addProcessor(colourMassDetectionProcessor)
                .build();
        FtcDashboard.getInstance().startCameraStream(visionPortal, 30);
        while(opModeInInit()) {
            setupLoop();
//            brightness.runPipeline();
//            telemetry.addData("Currently Recorded Position", brightness.getPosition());
//            telemetry.addData("Currently Recorded Position", mk(brightness.getPosition()));
//            telemetry.addData("It is", brightness.getColor());
            telemetry.addData("Currently Recorded Position", colourMassDetectionProcessor.getRecordedPropPosition());
            telemetry.addData("Camera State", visionPortal.getCameraState());
            telemetry.addData("Currently Detected Mass Center", "x: " + colourMassDetectionProcessor.getLargestContourX() + ", y: " + colourMassDetectionProcessor.getLargestContourY());
            telemetry.addData("Currently Detected Mass Area", colourMassDetectionProcessor.getLargestContourArea());
            telemetry.update();
        }
        waitForStart();
        if (isStopRequested()) return;
        if (visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
            visionPortal.stopLiveView();
            visionPortal.stopStreaming();
        }

        // gets the recorded prop position
        ColourMassDetectionProcessor.PropPositions recordedPropPositionL = colourMassDetectionProcessor.getRecordedPropPosition();

        // now we can use recordedPropPosition to determine where the prop is! if we never saw a prop, your recorded position will be UNFOUND.
        // if it is UNFOUND, you can manually set it to any of the other positions to guess
        if (recordedPropPositionL == ColourMassDetectionProcessor.PropPositions.UNFOUND) recordedPropPositionL = ColourMassDetectionProcessor.PropPositions.MIDDLE;

//        ColourMassDetectionProcessor.PropPositions recordedPropPositionL = mk(brightness.getPosition());

        onStarted(new ColourMassDetectionProcessor.Prop(recordedPropPositionL, this.name));

        // now we can use recordedPropPosition in our auto code to modify where we place the purple and yellow pixels
        onceStartedColor(new ColourMassDetectionProcessor.Prop(recordedPropPositionL, this.name));
        while(opModeIsActive() && !isStopRequested()) {
            onStartedColor(new ColourMassDetectionProcessor.Prop(recordedPropPositionL, this.name));

            robot.update();
            Storage.currentPose = robot.getPoseEstimate();
        }
        colourMassDetectionProcessor.close();
        visionPortal.close();
    }
    public ColourMassDetectionProcessor.PropPositions mk(int a) {
        switch(a) {
            case 0: return ColourMassDetectionProcessor.PropPositions.UNFOUND;
            case 1: return ColourMassDetectionProcessor.PropPositions.LEFT;
            case 2: return ColourMassDetectionProcessor.PropPositions.MIDDLE;
            case 3: return ColourMassDetectionProcessor.PropPositions.RIGHT;
        }
        return ColourMassDetectionProcessor.PropPositions.UNFOUND;
    }
    public void setup() {}
    public void setupLoop() {}
    public void onStarted(ColourMassDetectionProcessor.Prop detectedProp) {}
    public void onStartedColor(ColourMassDetectionProcessor.Prop detectedProp) {} // TO BE OVERRIDDEN IN ANY EXTENDED CLASSES
    public void onceStartedColor(ColourMassDetectionProcessor.Prop detectedProp) {} // TO BE OVERRIDDEN IN ANY EXTENDED CLASSES
}
