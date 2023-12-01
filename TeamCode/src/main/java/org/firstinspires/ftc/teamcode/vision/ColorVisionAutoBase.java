package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.vision.VisionPortal;
import org.opencv.core.Scalar;

import java.util.function.DoubleSupplier;

//@Disabled // remove this line to have this show up on your robot
//@Autonomous(name = "Backstage Red")
public class ColorVisionAutoBase extends LinearOpMode {
    private VisionPortal visionPortal;
    private ElapsedTime runtime = new ElapsedTime();
    static final double     COUNTS_PER_MOTOR_REV    = 384.5 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;
    private ColourMassDetectionProcessor colourMassDetectionProcessor;
    RobotHardware robot = new RobotHardware();
    Scalar lower = new Scalar(150, 100, 100); // the lower hsv threshold for your detection
    Scalar upper = new Scalar(180, 255, 255); // the upper hsv threshold for your detection
    DoubleSupplier minArea = () -> 100; // the minimum area for the detection to consider for your prop
    DoubleSupplier left = () -> 213;
    DoubleSupplier right = () -> 426;


    @Override
    public void runOpMode() {
        setup();
        robot.init(hardwareMap);
        colourMassDetectionProcessor = new ColourMassDetectionProcessor(this.lower, this.upper, this.minArea, this.left, this.right);
        visionPortal = new VisionPortal.Builder()
                .setCamera(robot.camera) // the camera on your robot is named "Webcam 1" by default
                .addProcessor(colourMassDetectionProcessor)
                .build();
        telemetry.addData("Currently Recorded Position", colourMassDetectionProcessor.getRecordedPropPosition());
        telemetry.addData("Camera State", visionPortal.getCameraState());
        telemetry.addData("Currently Detected Mass Center", "x: " + colourMassDetectionProcessor.getLargestContourX() + ", y: " + colourMassDetectionProcessor.getLargestContourY());
        telemetry.addData("Currently Detected Mass Area", colourMassDetectionProcessor.getLargestContourArea());

        waitForStart();
        if (visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
            visionPortal.stopLiveView();
            visionPortal.stopStreaming();
        }

        // gets the recorded prop position
        ColourMassDetectionProcessor.PropPositions recordedPropPosition = colourMassDetectionProcessor.getRecordedPropPosition();

        // now we can use recordedPropPosition to determine where the prop is! if we never saw a prop, your recorded position will be UNFOUND.
        // if it is UNFOUND, you can manually set it to any of the other positions to guess
        if (recordedPropPosition == ColourMassDetectionProcessor.PropPositions.UNFOUND) {
            recordedPropPosition = ColourMassDetectionProcessor.PropPositions.MIDDLE;
        }

        // now we can use recordedPropPosition in our auto code to modify where we place the purple and yellow pixels
        onStartedColor(recordedPropPosition);
        while(opModeIsActive()) {
            opModeActiveLoop();
            if (isStopRequested()) {
                stop();
            }
        }
        colourMassDetectionProcessor.close();
        visionPortal.close();
    }
    public void setup() {}
    public void opModeActiveLoop() {}
    public void onStartedColor(ColourMassDetectionProcessor.PropPositions propPos) {} // TO BE OVERRIDDEN IN ANY EXTENDED CLASSES

    public void encoderDrive(double speed,
                             double leftFinches,
                             double rightFinches,
                             double leftBinches,
                             double rightBinches,
                             double timeoutS) {
        int newLeftFTarget;
        int newRightFTarget;
        int newLeftBTarget;
        int newRightBTarget;

        robot.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Ensure that the opmode is still active
        newLeftFTarget = robot.frontRight.getCurrentPosition() + (int) (rightFinches * COUNTS_PER_INCH);
        newRightFTarget = robot.frontLeft.getCurrentPosition() + (int) (leftFinches * COUNTS_PER_INCH);
        newLeftBTarget = robot.backRight.getCurrentPosition() + (int) (rightBinches * COUNTS_PER_INCH);
        newRightBTarget = robot.backLeft.getCurrentPosition() + (int) (leftBinches * COUNTS_PER_INCH);

        robot.frontRight.setTargetPosition(newRightFTarget);
        robot.frontLeft.setTargetPosition(newLeftFTarget);
        robot.backRight.setTargetPosition(newRightBTarget);
        robot.backLeft.setTargetPosition(newLeftBTarget);

        // Turn On RUN_TO_POSITION
        robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // reset the timeout time and start motion.
        runtime.reset();
        robot.frontRight.setPower(Math.abs(speed));
        robot.frontLeft.setPower(Math.abs(speed));
        robot.backRight.setPower(Math.abs(speed));
        robot.backLeft.setPower(Math.abs(speed));

        // keep looping while we are still active, and there is time left, and both motors are running.
        // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
        // its target position, the motion will stop.  This is "safer" in the event that the robot will
        // always end the motion as soon as possible.
        // However, if you require that BOTH motors have finished their moves before the robot continues
        // onto the next step, use (isBusy() || isBusy()) in the loop test.


        while (opModeIsActive() &&
                (runtime.seconds() < timeoutS) &&
                (robot.frontLeft.isBusy() && robot.frontRight.isBusy() && robot.backLeft.isBusy() && robot.backRight.isBusy())) {
            telemetry.addData("Path1", "Running to %7d :%7d", newLeftFTarget, newRightFTarget, newLeftBTarget, newRightBTarget);
            telemetry.addData("Path2", "Running at %7d :%7d",
                    robot.frontLeft.getCurrentPosition(),
                    robot.frontRight.getCurrentPosition(),
                    robot.backLeft.getCurrentPosition(),
                    robot.backRight.getCurrentPosition());
            telemetry.update();
        }

        // Stop all motion;
        robot.frontRight.setPower(0);
        robot.frontLeft.setPower(0);
        robot.backRight.setPower(0);
        robot.backLeft.setPower(0);

        // Turn off RUN_TO_POSITION
        robot.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}
