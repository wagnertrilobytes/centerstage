/* Copyright (c) 2017 FIRST. All rights reserved.
         *
         * Redistribution and use in source and binary forms, with or without modification, * are permitted (subject to the limitations in the disclaimer below) provided that * the following conditions are met:
         *
         * Redistributions of source code must retain the above copyright notice, this list * of conditions and the following disclaimer.
         *
         * Redistributions in binary form must reproduce the above copyright notice, this * list of conditions and the following disclaimer in the documentation and/or * other materials provided with the distribution.
         *
         * Neither the name of FIRST nor the names of its contributors may be used to endorse or * promote products derived from this software without specific prior written permission. *
         * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
         * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
         * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
         * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
         * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
         * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
         * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
         * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
         * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
         * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
         * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */
        package org.firstinspires.ftc.teamcode;
import static org.firstinspires.ftc.teamcode.RobotHardware.WHEEL_DIAMETER_INCHES;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.util.Hardware;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * This file illustrates the concept of driving a path based on encoder counts. * It uses the common Pushbot hardware class to define the drive on the robot. * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 * otherwise you would use: PushbotAutoDriveByTime;
 *
 * This code ALSO requires that the drive Motors have been configured such that a positive * power command moves them forwards, and causes the encoders to count UP. *
 * The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 * that performs the actual movement.
 * This methods assumes that each movement is relative to the last stopping place. * There are other ways to perform encoder based moves, but this method is probably the simplest.
 * This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@Autonomous(name="BlueAutoPastTest", group="Pushbot")
public class BlueAutoPastTest extends LinearOpMode {
    /* Declare OpMode members. */
    RobotHardware robot = new RobotHardware(); // Use a Pushbot's hardware private ElapsedTime runtime = new ElapsedTime();
    static final double COUNTS_PER_MOTOR_REV = 384.5 ; // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0 ; // This is < 1.0 if geared UP static final double WHEEL_DIAMETER_INCHES = 4 ; // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION*1.266) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double Fast_SPEED = 0.7;
    static final double Slow_SPEED = 0.5;
    @Override
    public void runOpMode() {
        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap, this, false);
// Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders"); //
        telemetry.update();
        robot.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
// Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0", "Starting at %7d :%7d",
                robot.frontLeft.getCurrentPosition(),
                robot.frontRight.getCurrentPosition(),
                robot.backLeft.getCurrentPosition(),
                robot.backRight.getCurrentPosition());
        telemetry.update();
// Wait for the game to start (driver presses PLAY)
        waitForStart();
// Note: Reverse movement is obtained by setting a negative distance (not speed) encoderDrive(Slow_SPEED, -2, 2, 2, -2, 5.0);
        encoderDrive(Slow_SPEED, 10, 10, 10, 10, 5.0);
      //  encoderDrive(Slow_SPEED, -40, -40, -40, -40, 2.0);
       // encoderDrive(Slow_SPEED, -16, 16, 16, -16, 4.0);

        // The desired path in the example above is:

//* - Drive forward for 18 Inches
//* - Spin Duck Motor at -.65 speed for 3000 miliseconds //* - Strafe left for 32 inches
//* - Drive forward for 9 Inches
        sleep(1000); // pause for servos to move
        telemetry.addData("Path", "Complete");
        telemetry.update();
    }
    /*
     * Method to perform a relative move, based on encoder counts.
     * Encoders are not reset as the move is based on the current position. * Move will stop if any of three conditions occur:
     * 1) Move gets to the desired position
     * 2) Move runs out of time
     * 3) Driver stops the opmode running.
     */
    public void encoderDrive(double speed,
                             double leftFInches, double rightFInches,
                             double leftBInches, double rightBInches,
                             double timeoutS) {
        int newLeftFTarget;
        int newRightFTarget;
        int newLeftBTarget;
        int newRightBTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {


            // Determine new target position, and pass to motor controller
            newLeftFTarget = robot.frontLeft.getCurrentPosition() + (int)(leftFInches * COUNTS_PER_INCH);
            newRightFTarget = robot.frontRight.getCurrentPosition() + (int)(rightFInches * COUNTS_PER_INCH);
            newLeftBTarget = robot.backLeft.getCurrentPosition() + (int)(leftBInches * COUNTS_PER_INCH);
            newRightBTarget = robot.backRight.getCurrentPosition() + (int)(rightBInches * COUNTS_PER_INCH);
            robot.frontLeft.setTargetPosition(newLeftFTarget);
            robot.frontRight.setTargetPosition(newRightFTarget);
            robot.backLeft.setTargetPosition(newLeftBTarget);
            robot.backRight.setTargetPosition(newRightBTarget);

            // Turn On RUN_TO_POSITION
            robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            resetRuntime();
            robot.frontLeft.setPower(Math.abs(speed));
            robot.frontRight.setPower(Math.abs(speed));
            robot.backLeft.setPower(Math.abs(speed));
            robot.backRight.setPower(Math.abs(speed));
// keep looping while we are still active, and there is time left, and both motors are running.
// Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
// its target position, the motion will stop. This is "safer" in the event that the robot will // always end the motion as soon as possible.
// However, if you require that BOTH motors have finished their moves before the robot continues
// onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (getRuntime() < timeoutS) &&
                    (robot.frontLeft.isBusy() && robot.frontRight.isBusy() && robot.backLeft.isBusy() && robot.backRight.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newLeftFTarget,
                        newRightFTarget, newLeftBTarget, newRightBTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        robot.frontLeft.getCurrentPosition(),
                        robot.frontRight.getCurrentPosition(),
                        robot.backLeft.getCurrentPosition(),
                        robot.backRight.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backLeft.setPower(0);
            robot.backRight.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            // sleep(250); // optional pause after each move
        }
    }
}
