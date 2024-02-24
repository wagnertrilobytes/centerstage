package org.firstinspires.ftc.teamcode.opmodes.auto.encoderbased;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.SpicyBucketCR;
import org.firstinspires.ftc.teamcode.vision.ColorVisionAutoBase;
import org.firstinspires.ftc.teamcode.vision.ColourMassDetectionProcessor;
import org.opencv.core.Scalar;

@Autonomous(name = "Red Camera Simple Auto", group="Fallback")
@Disabled
public class CameraSimpleRefd extends ColorVisionAutoBase {
    private ElapsedTime runtime = new ElapsedTime();
    static final double     COUNTS_PER_MOTOR_REV    = 384.5 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;

        SampleMecanumDrive robot;
        SpicyBucketCR bucket = new SpicyBucketCR();
        Intake intake = new Intake();
    @Override
    public void setup() {
        robot = new SampleMecanumDrive(hardwareMap);
        this.lower = new Scalar(40, 100, 100); // the lower hsv threshold for your detection
        this.upper = new Scalar(180, 255, 255); // the upper hsv threshold for your detection
        this.minArea = () -> 2000; // the minimum area for the detection to consider for your prop
        this.left = () -> 213;
        this.right = () -> 426;

        bucket.init(hardwareMap);
        intake.init(hardwareMap);
        telemetry.addData("Starting at", "%7d :%7d",
                robot.frontRight.getCurrentPosition(),
                robot.frontLeft.getCurrentPosition(),
                robot.backRight.getCurrentPosition(),
                robot.backLeft.getCurrentPosition());
        telemetry.update();
    }

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

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller

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

    @Override
    public void onStarted(ColourMassDetectionProcessor.Prop propPosL) {
        telemetry.update();
        switch(propPosL.getPosition()) {
            case RIGHT:
                encoderDrive(0.5, -15, 15, 15, -15, 7);
                sleep(1000);
                encoderDrive(0.5, 22, 22, 22, 22, 7);
                sleep(1000);
                bucket.takeOut();
                intake.setPower(0.25, -1);
                sleep(1250);
                intake.stop();
                bucket.stop();
                break;
            case UNFOUND: // we can also just add the unfound case here to do fallthrough intstead of the overriding method above, whatever you prefer!
            case MIDDLE:
                // code to do if we saw the prop on the middle
                encoderDrive(0.5, 22, 22, 22, 22, 7);
                bucket.takeOut();
                intake.setPower(0.25, -1);
                sleep(1250);
                intake.stop();
                bucket.stop();
                sleep(1000);
                break;
            case LEFT:
                // code to do if we saw the prop on the right
                encoderDrive(0.5, 7, -7, 7, -7, 7);
                sleep(1000);
                encoderDrive(0.5, -15, 15, 15, -15, 7);
                bucket.takeOut();
                intake.setPower(0.25, -1);
                sleep(1250);
                intake.stop();
                bucket.stop();
                sleep(1000);
                break;
        }


    }
}
