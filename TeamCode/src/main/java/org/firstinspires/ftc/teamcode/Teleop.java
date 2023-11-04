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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="PowerPlay: Teleop", group="Linear Opmode")
//@Disabled
public class Teleop extends LinearOpMode {
    RobotHardware robot = new RobotHardware();

    // Declare OpMode mem bers.
    //private ElapsedTime runtime = new ElapsedTime();
    //private DcMotor leftDrive = null;
    //private DcMotor rightDrive = null;



    //@Override
    public void runOpMode() {
        robot.init(hardwareMap);
        telemetry.addData("Status", "Initialized");
        telemetry.update();




       // robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        //runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
           // robot.arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);




            double Speed = -gamepad1.left_stick_y;
            double Turn = -gamepad1.left_stick_x;
            double Strafe = -gamepad1.right_stick_x;
            double Slide = gamepad2.right_stick_y;
            double suck = gamepad2.left_stick_x;
            double flip = -gamepad2.left_stick_y;
            double MAX_SPEED = 1.0;



            double numFl = 0.8*Range.clip((+Speed - Turn - Strafe), -1, +1);
            double numFr = 0.8*Range.clip((+Speed + Turn + Strafe), -1, +1);
            double numBl = 0.8*Range.clip((+Speed + Turn - Strafe), -1, +1);
            double numBr = 0.8*Range.clip((+Speed - Turn + Strafe), -1, +1);
            double numUp = 0.10*Range.clip((Slide), -1, +1);
            double numFlip = 0.7*Range.clip((flip), -1, +1);
            //double numSuck = 0.2*Range.clip((+suck), -1, +1);

            if (gamepad2.b){
                robot.plane.setPosition(-0.7);
            }
            if (gamepad2.a){
                robot.hook.setPosition(-1.0);
            }

           /* if (gamepad2.b && rotation<2824)
                robot.arm.setTargetPosition(1000);
                robot.arm.getCurrentPosition();
                if (robot.arm.)
                robot.arm.setPower(.5);*/
            //Fabian Bafoonery

            //rotation values for height
            // small:negative -1875
            // medium: -3150
            // tall height: -4200




            // Show the elapsed game time and wheel power.
            //telemetry.addData("Status", "Run Time: " + runtime.toString());


            double frontLeftPower = robot.frontLeft.getPower();
            double frontRightPower = robot.frontRight.getPower();
            double backLeftPower = robot.backLeft.getPower();
            double backRightPower = robot.backRight.getPower();
            double slideLeftPower  = robot.slideLeft.getPower();
            double slideRightPower = robot.slideRight.getPower();
            //telemetry.addData("Arm height:", robot.arm.getCurrentPosition());
            telemetry.addData("Motors Power", "frontLeft (%.2f), frontRight (%.2f), backLeft (%.2f), backRight (%.2f), slideLeft (%.2f), slideRight (%.2f)", frontLeftPower, frontRightPower, backLeftPower,backRightPower, slideLeftPower, slideRightPower);
            telemetry.update();




            //robot.armB.setPower(numUp - MAX_SPEED + MAX_SPEED);
            robot.frontLeft.setPower(numFl - MAX_SPEED + MAX_SPEED);
            robot.frontRight.setPower(numFr - MAX_SPEED + MAX_SPEED);
            robot.backLeft.setPower(numBl - MAX_SPEED + MAX_SPEED);
            robot.backRight.setPower(numBr - MAX_SPEED + MAX_SPEED);
            //slides
            robot.intake.setPower(suck - 0.35 + 0.35);
            robot.claw.setPower(numFlip - MAX_SPEED  + MAX_SPEED);
            robot.slideLeft.setPower(numUp - MAX_SPEED + MAX_SPEED);
            robot.slideRight.setPower(numUp - MAX_SPEED + MAX_SPEED);






            boolean vroom = true;
            if(vroom == true)
            {
                while(gamepad1.dpad_down)
                {
                    robot.frontLeft.setPower(-1);
                    robot.frontRight.setPower(-1);
                    robot.backLeft.setPower(-1);
                    robot.backRight.setPower(-1);
                    vroom = false;
                }
                if(vroom == false)
                {
                    robot.frontLeft.setPower(0);
                    robot.frontRight.setPower(0);
                    robot.backLeft.setPower(0);
                    robot.backRight.setPower(0);
                }
            }
            boolean vroom2 = true;
            if(vroom2 == true)
            {
                while(gamepad1.dpad_up)
                {
                    robot.frontLeft.setPower(1);
                    robot.frontRight.setPower(1);
                    robot.backLeft.setPower(1);
                    robot.backRight.setPower(1);
                    vroom2 = false;
                }
                if(vroom2 == false)
                {
                    robot.frontLeft.setPower(0);
                    robot.frontRight.setPower(0);
                    robot.backLeft.setPower(0);
                    robot.backRight.setPower(0);
                }
            }

            robot.frontLeft.setPower(numFl - MAX_SPEED + MAX_SPEED);
            if (robot.backLeft != null) {
                robot.backLeft.setPower(numBl - MAX_SPEED + MAX_SPEED);
            }
            robot.frontRight.setPower(numFr - MAX_SPEED + MAX_SPEED);
            if (robot.backRight != null) {
                robot.backRight.setPower(numBr - MAX_SPEED + MAX_SPEED);
            }

            robot.frontLeft.setPower(numFl - MAX_SPEED + MAX_SPEED);
            if (robot.backLeft != null) {
                robot.backLeft.setPower(numBl - MAX_SPEED + MAX_SPEED);
            }
            robot.frontRight.setPower(numFr - MAX_SPEED + MAX_SPEED);
            if (robot.backRight != null) {
                robot.backRight.setPower(numBr - MAX_SPEED + MAX_SPEED);
            }




            telemetry.update();
        }
    }
}
