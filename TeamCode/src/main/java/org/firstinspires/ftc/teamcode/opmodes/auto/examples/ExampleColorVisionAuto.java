package org.firstinspires.ftc.teamcode.opmodes.auto.examples;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.vision.ColorVisionAutoBase;
import org.firstinspires.ftc.teamcode.vision.ColourMassDetectionProcessor;
import org.opencv.core.Scalar;

@Autonomous(name = "Example Color Vision Autonomous")
@Disabled
public class ExampleColorVisionAuto extends ColorVisionAutoBase {
    @Override
    public void setup() {
        this.lower = new Scalar(150, 100, 100); // the lower hsv threshold for your detection
        this.upper = new Scalar(180, 255, 255); // the upper hsv threshold for your detection
        this.minArea = () -> 100; // the minimum area for the detection to consider for your prop
        this.left = () -> 213;
        this.right = () -> 426;
    }

    @Override
    public void onStarted(ColourMassDetectionProcessor.Prop prop) {
        telemetry.addData("Test", robot.lastPropPos.toString());
        telemetry.update();
    }

    @Override
    public void onStartedColor(ColourMassDetectionProcessor.Prop propPosL) {
        switch(propPosL.getPosition()) {
            case LEFT:
                telemetry.addData("Seen", "Left");
                break;
            case UNFOUND: // we can also just add the unfound case here to do fallthrough intstead of the overriding method above, whatever you prefer!
            case MIDDLE:
                // code to do if we saw the prop on the middle
                telemetry.addData("Seen", "Middle");
                break;
            case RIGHT:
                // code to do if we saw the prop on the right
                telemetry.addData("Seen", "Right");
                break;
        }
    }
}
