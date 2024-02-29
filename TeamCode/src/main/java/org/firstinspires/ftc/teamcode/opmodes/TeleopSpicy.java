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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.helpers.Storage;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.CounterRoller;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.PlaneLauncher;
import org.firstinspires.ftc.teamcode.subsystems.Slides;
import org.firstinspires.ftc.teamcode.subsystems.SpicyBucketCR;

@TeleOp(name="Centerstage: Teleop PizzaBox Lives On", group="Main")
public class TeleopSpicy extends LinearOpMode {
    SpicyBucketCR bucket = new SpicyBucketCR();
    Intake intake = new Intake();
    PlaneLauncher plane = new PlaneLauncher();
    Slides slides = new Slides();
    CounterRoller roller = new CounterRoller();
    SampleMecanumDrive robot;
    int bucketArmPower = 0;
    @Override
    public void runOpMode() throws InterruptedException {
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

        waitForStart();
        while (!isStopRequested()) {
            robot.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );

            robot.update();

            double slidePowerMultiplier = 0.65;
            double intakePowerMultiplier = 0.4;

            slides.setPower(slidePowerMultiplier * Range.clip(gamepad2.right_stick_y, -1, +1));

            if (gamepad2.left_trigger > 0.3) {
                intake.setPower(-gamepad2.left_trigger, intakePowerMultiplier);
                bucket.takeOut();
                roller.spinForward();
            }
            if (gamepad2.right_trigger > 0.3) {
                intake.setPower(gamepad2.right_trigger, intakePowerMultiplier);
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

            if (gamepad2.dpad_left) bucketArmPower = -1;
            if (gamepad2.dpad_right) bucketArmPower = 1;
            if (!gamepad2.dpad_left && !gamepad2.dpad_right) bucketArmPower = 0;

            bucket.setBucketArmPower(bucketArmPower);
        }
    }
}
