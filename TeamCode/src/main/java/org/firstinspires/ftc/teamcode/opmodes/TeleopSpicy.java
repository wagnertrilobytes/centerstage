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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.teamcode.helpers.Storage;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.CounterRoller;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.PlaneLauncher;
import org.firstinspires.ftc.teamcode.subsystems.Slides;
import org.firstinspires.ftc.teamcode.subsystems.SpicyBucket;
import org.firstinspires.ftc.teamcode.subsystems.SpicyBucketCR;
import org.firstinspires.ftc.teamcode.subsystems.YeOldeBucket;
import org.firstinspires.ftc.teamcode.vision.ColorBrightness;


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

@TeleOp(name="Centerstage: Teleop PizzaBox Lives On", group="Main")
//@Disabled
public class TeleopSpicy extends OpMode {
//    YeOldeBucket bucket = new YeOldeBucket();
    SpicyBucketCR bucket = new SpicyBucketCR();
    Intake intake = new Intake();
    PlaneLauncher plane = new PlaneLauncher();
    Slides slides = new Slides();
    CounterRoller roller = new CounterRoller();
    SampleMecanumDrive robot;
    ColorBrightness cb;
    @Override
    public void init() {
        robot = new SampleMecanumDrive(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        bucket.init(hardwareMap);
        intake.init(hardwareMap);
        plane.init(hardwareMap);
        slides.init(hardwareMap);
        roller.init(hardwareMap);

        if (Storage.currentPose != null) robot.setPoseEstimate(Storage.currentPose);

        telemetry.addData("Status", "Initialized");

        telemetry.update();
    }
    int power = 0;
    @Override
    public void loop() {
        bucket.run(gamepad1, gamepad2, telemetry);
        intake.run(gamepad1, gamepad2, telemetry);
        plane.run(gamepad1, gamepad2, telemetry);
        slides.run(gamepad1, gamepad2, telemetry);

        double Speed = -gamepad1.left_stick_y;
        double Turn = gamepad1.left_stick_x;
        double Strafe = -gamepad1.right_stick_x;
        double Slide = -gamepad2.right_stick_y;
        double MAX_SPEED = 1.0;

        double numFl = (0.75*Range.clip((+Speed + Turn - Strafe), -1, +1)) - MAX_SPEED + MAX_SPEED;
        double numFr = (0.75*Range.clip((+Speed + Turn + Strafe), -1, +1)) - MAX_SPEED + MAX_SPEED;
        double numBl = (0.75*Range.clip((+Speed - Turn - Strafe), -1, +1)) - MAX_SPEED + MAX_SPEED;
        double numBr = (0.75*Range.clip((+Speed - Turn + Strafe), -1, +1)) - MAX_SPEED + MAX_SPEED;

        robot.setMotorPowers(numFl, numBl, numFr, numBr);

        double numUp = 0.65*Range.clip((Slide), -1, +1);

        slides.setPower((numUp) - MAX_SPEED + MAX_SPEED);

        double iSM = 0.4;
        if (gamepad2.left_trigger > 0.3) {
            intake.setPower(-gamepad2.left_trigger, iSM);
            bucket.takeOut();
            roller.spinForward();
        }
        if (gamepad2.right_trigger > 0.3) {
            intake.setPower(gamepad2.right_trigger, iSM);
            bucket.takeIn();
            roller.spinBackward();
        }
        if (gamepad2.right_trigger < 0.3 && gamepad2.left_trigger < 0.3) {
            intake.stop();
            bucket.stop();
            roller.stop();
        }
        if (gamepad2.y) plane.sendPlane();
        else plane.takePlaneBack();
        if (gamepad2.a) bucket.dropOnePixel();

        if (gamepad1.dpad_up) robot.setMotorPowers(1, 1, 1, 1);
        if (gamepad1.dpad_down) robot.setMotorPowers(-1, -1, -1, -1);


        if (gamepad2.dpad_up) bucket.setSlideArmPower(1);
        if (gamepad2.dpad_down) bucket.setSlideArmPower(-1);
        if (!gamepad2.dpad_up && !gamepad2.dpad_down) bucket.setSlideArmPower(0);

        if (gamepad2.dpad_left) power = -1;
        if (gamepad2.dpad_right) power = 1;
        if (!gamepad2.dpad_left && !gamepad2.dpad_right) power = 0;

        bucket.setBucketArmPower(power);

        robot.update();

        telemetry.addData("Hooligan", "Activity");
        telemetry.addData("Front Left", fmt(robot.frontLeft));
        telemetry.addData("Front Right", fmt(robot.frontRight));
        telemetry.addData("Back Left", fmt(robot.backLeft));
        telemetry.addData("Back Right", fmt(robot.backRight));

        telemetry.update();
    }
    public String fmt(DcMotor mot) {
        return mot.getCurrentPosition() + "@" + mot.getPower() + "," + mot.getPortNumber();
    }

    public String fmt(Servo mot) {
        return mot.getPosition() + ":" + mot.getDirection() + "," + mot.getPortNumber();
    }
}
