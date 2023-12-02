package org.firstinspires.ftc.teamcode.vision;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.opencv.core.Scalar;

@Config
@Autonomous(name = "Blue Tuning", group = "AutoTuning")
public class BlueTuneTest extends ColorVisionAutoBase {
    public static double lower0 = 40;
    public static double lower1 = 100;
    public static double lower2 = 100;

    public static double upper0 = 60;
    public static double upper1 = 255;
    public static double upper2 = 255;

    public FtcDashboard dashboard = FtcDashboard.getInstance();
    @Override
    public void setup() {
        this.lower = new Scalar(lower0, lower1, lower2);
        this.upper = new Scalar(upper0, upper1, upper2); // the upper hsv threshold for your detection
        this.minArea = () -> 100; // the minimum area for the detection to consider for your prop
        this.left = () -> 213;
        this.right = () -> 426;
    }

    @Override
    public void setupLoop() {
        TelemetryPacket packet = new TelemetryPacket();
        packet.fieldOverlay().clear();
        dashboard.sendTelemetryPacket(packet);
        TelemetryPacket packetToo = new TelemetryPacket();
        packet.fieldOverlay()
                .fillText(this.colourMassDetectionProcessor.getRecordedPropPosition().toString(), 0, 0, "8px Arial", -Math.toRadians(45), false);
        dashboard.sendTelemetryPacket(packetToo);
    }

    @Override
    public void opModeActiveLoop() {
        telemetry.addData("Uhh", "This OpMode is not meant to be ran, only initialized. Change each value in FTC Dashboard until the camera can see it [marker] at any position (left, middle, right)");
        telemetry.addData("thumsb up emoji", ":+1: (:thumbsup:)");
    }
}
