package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.vision.ColorVisionAutoBase;
import org.firstinspires.ftc.teamcode.vision.ColourMassDetectionProcessor;
import org.firstinspires.ftc.teamcode.vision.ColourMassDetectionProcessorTwo;
import org.opencv.core.Scalar;

@Autonomous(name = "Backstage Red", group="Backstage")
public class BackstageRed extends ColorVisionAutoBase {
    @Override
    public void setup() {
        this.lower = new Scalar(150, 100, 100); // the lower hsv threshold for your detection
        this.upper = new Scalar(180, 255, 255); // the upper hsv threshold for your detection
        this.minArea = () -> 100; // the minimum area for the detection to consider for your prop
        this.left = () -> 213;
        this.right = () -> 426;
    }

    double speed = 0.4;

    @Override
    public void onStartedColor(ColourMassDetectionProcessor.PropPositions propPosL, ColourMassDetectionProcessorTwo.PropPositions propPosR) {
        encoderDrive(speed,
                -6,
                -6,
                -6,
                -6,
                7.0);
        // now we can use recordedPropPosition in our auto code to modify where we place the purple and yellow pixels
        switch (propPosL) {
            case LEFT:
                // code to do if we saw the prop on the left
                encoderDrive(speed + 0.3,
                        -30,
                        -30,
                        -30,
                        -30,
                        7.0);
                encoderDrive(speed,
                        -robot.TILE_LEN - 7,
                        robot.TILE_LEN - 7,
                        robot.TILE_LEN - 7,
                        -robot.TILE_LEN - 7,
                        7.0);
                robot.intake.setPower(-0.4);
                sleep(800);
                robot.intake.setPower(0);
                encoderDrive(speed + 0.3,
                        robot.TILE_LEN * 3.2,
                        -robot.TILE_LEN * 3.2,
                        -robot.TILE_LEN * 3.2,
                        robot.TILE_LEN * 3.2,
                        7.0);
                break;
            case UNFOUND: // we can also just add the unfound case here to do fallthrough intstead of the overriding method above, whatever you prefer!
            case MIDDLE:
                // code to do if we saw the prop on the middle
                encoderDrive(speed,
                        -20,
                        -20,
                        -20,
                        -20,
                        7.0);
                encoderDrive(speed,
                        -robot.TILE_LEN * 0.35,
                        robot.TILE_LEN * 0.35,
                        robot.TILE_LEN * 0.35,
                        -robot.TILE_LEN * 0.35,
                        7.0);
                encoderDrive(speed,
                        -15,
                        -15,
                        -15,
                        -15,
                        7.0);
                robot.intake.setPower(-0.25);
                sleep(1500);
                robot.intake.setPower(0);
                encoderDrive(speed,
                        12,
                        12,
                        12,
                        12,
                        7.0);
                encoderDrive(0.75,
                        65,
                        -65,
                        -65,
                        65,
                        7.0);
                break;
            case RIGHT:
                // code to do if we saw the prop on the right
                encoderDrive(speed,
                        -17,
                        -17,
                        -17,
                        -17,
                        7.0);
                encoderDrive(speed,
                        16,
                        -16,
                        -16,
                        16,
                        7.0);
                robot.intake.setPower(-0.4);
                sleep(800);
                robot.intake.setPower(0);
                encoderDrive(speed,
                        42,
                        -42,
                        -42,
                        42,
                        7.0);
                break;
        }
    }
}
