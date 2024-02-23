package org.firstinspires.ftc.teamcode.opmodes.tuners;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.vision.ColorVisionAutoBase;
import org.firstinspires.ftc.teamcode.vision.ColourMassDetectionProcessor;
import org.opencv.core.Scalar;

@Config
@Autonomous(name = "Color Tuning", group = "Tuners")
public class ColorTuneTest extends ColorVisionAutoBase {
    public static double lower0 = 40;
    public static double lower1 = 100;
    public static double lower2 = 100;

    public static double upper0 = 180;
    public static double upper1 = 255;
    public static double upper2 = 255;
    public static double minAreaC = 8000;
    public static double leftC = 213;
    public static double rightC = 416;

    public FtcDashboard dashboard = FtcDashboard.getInstance();
    @Override
    public void setup() {
        this.lower = new Scalar(lower0, lower1, lower2);
        this.upper = new Scalar(upper0, upper1, upper2); // the upper hsv threshold for your detection
        this.minArea = () -> minAreaC; // the minimum area for the detection to consider for your prop
        this.left = () -> leftC;
        this.right = () -> rightC;
    }

    @Override
    public void setupLoop() {

    }


    @Override
    public void onStartedColor(ColourMassDetectionProcessor.Prop detectedProp) {
        telemetry.addData("Uhh", "This OpMode is not meant to be ran, only initialized. Change each value in FTC Dashboard until the camera can see it [marker] at any position (left, middle, right)");
        telemetry.addData("thumsb up emoji", ":+1: (:thumbsup:)");
    }
}
