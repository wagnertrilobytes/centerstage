package org.firstinspires.ftc.teamcode.opmodes.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

@Config
@Autonomous(name = "Testing: servo as CR")
public class TestServoAsCR extends LinearOpMode {
    public static double powerPos = 0.1;
    public static double powerNeg = -0.1;
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        waitForStart();
        while(opModeIsActive() && !isStopRequested()) {
            if (gamepad1.a) {
                drive.testServo.setPower(powerPos);
            }
            if (gamepad1.b) {
                drive.testServo.setPower(powerNeg);
            }
            if (gamepad1.y) {
                drive.testServo.zero();
            }
            telemetry.addData("Gp1 A", "Servo PWR +1");
            telemetry.addData("Gp1 B", "Servo PWR -1");
            telemetry.addData("Gp1 Y", "Servo POS 0");
//            telemetry.addData("POS", drive.clawLeft.getServo().getPosition());
//            telemetry.addData("LAST POWER ", drive.clawLeft.getLastPower());
            telemetry.update();
        }
    }
}
