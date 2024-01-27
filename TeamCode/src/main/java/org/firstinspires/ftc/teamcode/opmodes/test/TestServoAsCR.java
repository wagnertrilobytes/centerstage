package org.firstinspires.ftc.teamcode.opmodes.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.apache.commons.math3.util.IntegerSequence;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

import java.time.temporal.ValueRange;

@Config
@Autonomous(name = "Testing: servo as CR")
@Disabled
public class TestServoAsCR extends LinearOpMode {
    public static double powerPos = 1.01;
    public static double powerNeg = 0.01;
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        waitForStart();
        double turnAngle = 0;
        while(opModeIsActive() && !isStopRequested()) {
                double min = drive.testServo.min;
                double max = drive.testServo.max;
                double val = turnAngle;
                if (turnAngle > max) turnAngle = 360;
                if (turnAngle < min) turnAngle = 0;
            drive.testServo.turnToAngle(turnAngle);
            if (gamepad2.left_stick_y < 0.2) {
                turnAngle += gamepad2.left_stick_y;
            } else {
                turnAngle += gamepad2.left_stick_y;
            }
            telemetry.addData("Gp1 A", "Servo DEG ROT ++");
            telemetry.addData("Gp1 B", "Servo DEG ROT --");
            telemetry.addData("Gp1 Y", "Servo POS 0");
            telemetry.addData("POS", turnAngle);
            telemetry.addData("MAX", drive.testServo.max);
            telemetry.addData("GP", gamepad2.left_stick_y);
//            telemetry.addData("LAST POWER ", drive.testServo.getLastPower());
            telemetry.update();
//            drive.testServo.loop();
        }
    }
}
