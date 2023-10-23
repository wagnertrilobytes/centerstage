package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class RobotAutonomousFunctions {
    private final ElapsedTime runtime = new ElapsedTime();
    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;
    RobotHardware robot;
    public RobotAutonomousFunctions(RobotHardware robot) {
        this.robot = robot;
        robot.telemetry.addData("RAF", "Initialized");
    }

    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the OpMode running.
     */
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS, boolean opModeIsActive) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the OpMode is still active
        if (opModeIsActive) {

            // Determine new target position, and pass to motor controller
            int leftCounted = (int)(leftInches * COUNTS_PER_INCH);
            int rightCounted = (int)(rightInches * COUNTS_PER_INCH);
            newLeftTarget = robot.frontLeft.getCurrentPosition() + leftCounted;
            newRightTarget = robot.frontRight.getCurrentPosition() + rightCounted;
            robot.frontLeft.setTargetPosition(newLeftTarget);
            robot.backLeft.setTargetPosition(newLeftTarget);
            robot.frontRight.setTargetPosition(newRightTarget);
            robot.backRight.setTargetPosition(newRightTarget);

            for (DcMotor e : robot.motors)
            {
                e.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

            // reset the timeout time and start motion.
            runtime.reset();
            for (DcMotor e : robot.motors)
            {
                e.setPower(Math.abs(speed));
            }

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.frontLeft.isBusy() && robot.frontRight.isBusy()) &&
                    (robot.frontRight.isBusy() && robot.frontLeft.isBusy())
            ) {

                // Display it for the driver.
                robot.telemetry.addData("Running to",  " %7d :%7d", newLeftTarget,  newRightTarget);
                robot.telemetry.addData("Currently at",  " at fl%7d fr%7d bl%7d br%7d",
                        robot.frontLeft.getCurrentPosition(), robot.frontRight.getCurrentPosition(),
                        robot.backLeft.getCurrentPosition(), robot.backRight.getCurrentPosition());
                robot.telemetry.update();
            }

            // Stop all motion;
            robot.setAllPowerSpec(0,0,0,0);

            // Turn off RUN_TO_POSITION
            for (DcMotor e : robot.motors)
            {
                e.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }

            robot._opMode.sleep(250);   // optional pause after each move.
        }
    }
}
