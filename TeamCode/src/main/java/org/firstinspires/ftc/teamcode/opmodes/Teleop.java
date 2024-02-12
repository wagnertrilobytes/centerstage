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

package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.PlaneLauncher;
import org.firstinspires.ftc.teamcode.subsystems.Slides;
import org.firstinspires.ftc.teamcode.subsystems.SpicyBucket;
import org.firstinspires.ftc.teamcode.subsystems.YeOldeBucket;


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

@TeleOp(name="CenterStage: Teleop", group="Main")
@Disabled
public class Teleop extends LinearOpMode {
    YeOldeBucket bucket = new YeOldeBucket();
    Intake intake = new Intake();
    PlaneLauncher plane = new PlaneLauncher();
    Slides slides = new Slides();
    public void runOpMode() {
        SampleMecanumDrive robot = new SampleMecanumDrive(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        bucket.init(hardwareMap);
        intake.init(hardwareMap);
        plane.init(hardwareMap);
        slides.init(hardwareMap);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        double turnAngle = bucket.minArmAngle();
        while (opModeIsActive() && !isStopRequested()) {
            bucket.run(gamepad1, gamepad2, telemetry);
            intake.run(gamepad1, gamepad2, telemetry);
            plane.run(gamepad1, gamepad2, telemetry);
            slides.run(gamepad1, gamepad2, telemetry);

            double Speed = -gamepad1.left_stick_y;
            double Turn = gamepad1.right_stick_x;
            double Strafe = gamepad1.left_stick_x;
            double Slide = -gamepad2.right_stick_y;
            double MAX_SPEED = 1.0;

            double numFl = (0.75*Range.clip((+Speed + Turn - Strafe), -1, +1)) - MAX_SPEED + MAX_SPEED;
            double numFr = (0.75*Range.clip((+Speed + Turn + Strafe), -1, +1)) - MAX_SPEED + MAX_SPEED;
            double numBl = (0.75*Range.clip((+Speed - Turn - Strafe), -1, +1)) - MAX_SPEED + MAX_SPEED;
            double numBr = (0.75*Range.clip((+Speed - Turn + Strafe), -1, +1)) - MAX_SPEED + MAX_SPEED;

            robot.setMotorPowers(numFl, numBl, numFr, numBr);

            double numUp = 0.5*Range.clip((Slide), -1, +1);

            slides.setPower((numUp) - MAX_SPEED + MAX_SPEED);

            double iSM = 1;
            if (gamepad2.left_trigger > 0.3)  intake.setPower(-gamepad2.left_trigger, 1);
            if (gamepad2.right_trigger > 0.3) intake.setPower(gamepad2.right_trigger, iSM);
            if (gamepad2.right_trigger < 0.3 && gamepad2.left_trigger < 0.3) intake.stop();
            if (gamepad2.y) plane.sendPlane();
            else plane.takePlaneBack();

            turnAngle += -gamepad2.left_stick_y * 4;
            if (turnAngle > bucket.maxArmAngle()) turnAngle = bucket.maxArmAngle();
            if (turnAngle < bucket.minArmAngle()) turnAngle = bucket.minArmAngle();

            if (Math.abs(gamepad1.left_stick_x) > 0.1 ||
                Math.abs(gamepad1.left_stick_y) > 0.1 ||
                Math.abs(gamepad1.right_stick_x) > 0.1 ||
                Math.abs(gamepad1.right_stick_y) > 0.1 &&
                !(gamepad2.right_trigger > 0.1 || gamepad2.left_trigger > 0.1)
            ) {
                if(turnAngle <= bucket.minArmAngle() + 7 && !(gamepad2.right_trigger > 0 || gamepad2.left_trigger > 0)) bucket.turnTo(9);
            } else {
                bucket.turnTo(turnAngle);
            }
            if (gamepad1.dpad_up) robot.setMotorPowers(1, 1, 1, 1);
            if (gamepad1.dpad_down) robot.setMotorPowers(-1, -1, -1, -1);

            telemetry.addData("Hooligan", "Activity");
            telemetry.addData("Front Left", fmt(robot.frontLeft));
            telemetry.addData("Front Right", fmt(robot.frontRight));
            telemetry.addData("Back Left", fmt(robot.backLeft));
            telemetry.addData("Back Right", fmt(robot.backRight));

            telemetry.update();
        }
    }
    public String fmt(DcMotor mot) {
        return mot.getCurrentPosition() + "@" + mot.getPower() + "," + mot.getPortNumber();
    }

    public String fmt(Servo mot) {
        return mot.getPosition() + ":" + mot.getDirection() + "," + mot.getPortNumber();
    }
}
