/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
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
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="CenterStage: Teleop", group="Linear OpMode")
//@Disabled
public class Teleop extends LinearOpMode {
    RobotHardware robot = new RobotHardware();

    // Declare OpMode mem bers.
    public double MAX_SPEED = 1.0;
    //@Override
    public void runOpMode() {
        robot.init(hardwareMap);

        waitForStart(); // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Fabian Bafoonery", "Hardware initialized.");
        telemetry.update();
        while (opModeIsActive()) { // run until the end of the match (driver presses STOP)
            double Speed = gamepad1.left_stick_y;
            double Turn = gamepad1.left_stick_x;
            double Strafe = gamepad1.right_stick_x;
            double Slide = gamepad2.right_stick_y;
            double numUp = 0.10*Range.clip((-Slide), -1, +1);
            //Fabian Bafoonery

            //rotation values for height
            // small:  -1875
            // medium: -3150
            // tall:   -4200

            // double frontLeftPower = robot.frontLeft.getPower();
            // double frontRightPower = robot.frontRight.getPower();
            // double backLeftPower = robot.backLeft.getPower();
            // double backRightPower = robot.backRight.getPower();
            // double planePower = robot.plane.getPower();
            // double armPower = robot.arm.getPower();
            // telemetry.addData("Motors Power", "frontLeft (%.2f), frontRight (%.2f), backLeft (%.2f), backRight (%.2f), arm (%.2f)", frontLeftPower, frontRightPower, backLeftPower,backRightPower, armPower);

            //vroom drivey
            robot.driveRobot(Speed, Turn, Strafe);
            robot.slideLeft.setPower(numUp - robot.SLIDE_SPEED + robot.SLIDE_SPEED);
            robot.slideRight.setPower(numUp - robot.SLIDE_SPEED + robot.SLIDE_SPEED);

            /* Controls */

            // plane shooty
            robot.addPowerOnButtonPress(gamepad2.left_bumper, robot.plane, 1, 0);

            // chicken flingy
            robot.addPowerOnButtonPress(gamepad2.left_bumper, robot.drop, -1, 0);

            //intake grabby (in)
//            robot.addPowerOnButtonPress(gamepad2.dpad_up, robot.intake, 1, 0);
            if (gamepad2.dpad_up) {
                robot.intake.setPower(1);
            }
            if (gamepad2.dpad_down) {
                robot.intake.setPower(-1);
            }
            if (gamepad2.dpad_right) {
                robot.intake.setPower(0);
            }

            //intake spitty (out)
//            robot.addPowerOnButtonPress(gamepad2.dpad_down, robot.intake, -1, 0);

            //vroom drivey 2: electric boogaloo
            if (gamepad1.dpad_down) {
                robot.setAllPowerSpec(1,1,1,1);
            } else {
                robot.setAllPowerSpec(0,0,0,0);
            }

            if (gamepad1.dpad_up) {
                robot.setAllPowerSpec(-1,-1,-1,-1);
            } else {
                robot.setAllPowerSpec(0,0,0,0);
            }

        }
    }
}
