
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

        package org.firstinspires.ftc.teamcode.oldtest;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.util.Hardware;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

/**
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 * The code is structured as a LinearOpMode
 *
 * This particular OpMode executes a POV Game style Teleop for a PushBot
 * In this mode the left stick moves the robot FWD and back, the Right stick turns left and right.
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Game TeleOP", group="Old Tests")

public class GameTeleop extends LinearOpMode {

    /* Declare OpMode members. */
    GameHardware robot           = new GameHardware();   // Use a Pushbot's hardware
    double          clawOffset      = 0;                       // Servo mid position
    final double    CLAW_SPEED      = 0.02 ;                   // sets rate to move servo

    @Override
    public void runOpMode() {
        //using mecanum wheels you have 3 directions you can go, create variables for all 3 directions
        double forward;
        double turn;
        double strafe;

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap, this);//The name in parenthese MUST match the name of your hardware map

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "l?");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        robot.ArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.ArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            //Assign directions to gamepad controllers
            forward = -gamepad1.left_stick_y;
            turn = gamepad1.right_stick_x;
            strafe = gamepad1.left_stick_x;

            //how the robot calculates direction using what you do on the gamepad
            robot.LeftFront.setPower(turn+(forward+strafe));
            robot.RightFront.setPower(-turn+(forward-strafe));
            robot.LeftBack.setPower(turn+(forward-strafe));
            robot.RightBack.setPower(-turn+(forward+strafe));


            //We used this to figure out how high the arm needed to lift and show that on the driver station
            double rotation=robot.ArmMotor.getCurrentPosition();
            telemetry.addData("rotation", "%.2f", rotation);
            telemetry.update();

            //The bumpers are boolean meaning active or inactive so we set a specific power to each bumper if active
            if(gamepad1.right_bumper)
            {
                robot.IntakeMotor.setPower(.75);
            }
            else if(gamepad1.left_bumper)
            {
                robot.IntakeMotor.setPower(-.75);
            }
            else
            {
                robot.IntakeMotor.setPower(0);




                if(gamepad2.right_bumper)
                {
                    robot.IntakeMotor.setPower(1);
                }
                else if(gamepad2.left_bumper)
                {
                    robot.IntakeMotor.setPower(-1);
                }
                else
                {
                    robot.IntakeMotor.setPower(0);
                }



                if(gamepad2.dpad_up && rotation<2824)//We used && (AND) logic to say if a button is pushed and the encoder on the motor is below a certain amount then it can move up.
                {
                    robot.ArmMotor.setPower(.75);
                    robot.ArmMotorToo.setPower(.75);
                }
                else if(gamepad2.dpad_down && rotation>100)
                {
                    robot.ArmMotor.setPower(-.75);
                    robot.ArmMotorToo.setPower(-.75);
                }
                else if(gamepad2.b)
                {
                    robot.ArmMotor.setPower(-.5);
                    robot.ArmMotorToo.setPower(-.5);
                }
                else
                {
                    robot.ArmMotor.setPower(0);
                    robot.ArmMotorToo.setPower(0);
                }

                // Pace this loop so jaw action is reasonable speed.
                sleep(50);
            }
        }
    }
}


