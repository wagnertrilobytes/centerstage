package org.firstinspires.ftc.teamcode.vision;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.opencv.core.Scalar;

@Config
@Autonomous(name = "Blue Tuning", group = "AutoTuning")
public class BlueTuneTest extends ColorVisionAutoBase {
    public double lower0 = 150;
    public double lower1 = 100;
    public double lower2 = 100;

    public double upper0 = 180;
    public double upper1 = 255;
    public double upper2 = 255;
    @Override
    public void setup() {
        this.lower = new Scalar(lower0, lower1, lower2);
        this.upper = new Scalar(upper0, upper1, upper2); // the upper hsv threshold for your detection
        this.minArea = () -> 100; // the minimum area for the detection to consider for your prop
        this.left = () -> 213;
        this.right = () -> 426;
    }

    @Override
    public void onStartedColor(ColourMassDetectionProcessor.PropPositions propPos) {
        encoderDrive(0.5,
                -6,
                -6,
                -6,
                -6,
                7.0);
        // now we can use recordedPropPosition in our auto code to modify where we place the purple and yellow pixels
        switch (propPos) {
            case LEFT:
                encoderDrive(0.5,
                        -robot.TILE_LEN * 0.7,
                        robot.TILE_LEN * 0.7,
                        robot.TILE_LEN * 0.7,
                        -robot.TILE_LEN * 0.7,
                        7.0);
                // code to do if we saw the prop on the left
                encoderDrive(0.75,
                        -25,
                        -25,
                        -25,
                        -25,
                        7.0);
                robot.intake.setPower(-0.4);
                sleep(800);
                robot.intake.setPower(0);
                encoderDrive(0.25,
                        12,
                        12,
                        12,
                        12,
                        7.0);
                break;
            case UNFOUND: // we can also just add the unfound case here to do fallthrough intstead of the overriding method above, whatever you prefer!
            case MIDDLE:
                // code to do if we saw the prop on the middle
                encoderDrive(1,
                        -30,
                        -30,
                        -30,
                        -30,
                        7.0);
                encoderDrive(0.5,
                        -robot.TILE_LEN * 0.25,
                        robot.TILE_LEN * 0.25,
                        robot.TILE_LEN * 0.25,
                        -robot.TILE_LEN * 0.25,
                        7.0);
                encoderDrive(0.50,
                        -5,
                        -5,
                        -5,
                        -5,
                        7.0);
                robot.intake.setPower(-0.4);
                sleep(1000);
                robot.intake.setPower(0);
                encoderDrive(0.25,
                        12,
                        12,
                        12,
                        12,
                        7.0);
                break;
            case RIGHT:
                // code to do if we saw the prop on the right
                encoderDrive(0.4, -15, 15, -15, 15, 7.0);
                break;
        }
    }
}
