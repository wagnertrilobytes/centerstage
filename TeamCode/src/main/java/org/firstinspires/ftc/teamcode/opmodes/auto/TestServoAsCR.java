package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

@Autonomous(name = "Testing: servo as CR")
public class TestServoAsCR extends LinearOpMode {
    public static double powerPos = 0.0001;
    public static double powerNeg = 0.0001;
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        waitForStart();
        while(opModeIsActive() && !isStopRequested()) {
            if (gamepad1.a) {
                drive.clwLeft.positive(powerPos);
            }
            if (gamepad1.b) {
                drive.clwLeft.negative(powerNeg);
            }
            if (gamepad1.y) {
                drive.clwLeft.zero();
            }
            telemetry.addData("Gp1 A", "Servo PWR +1");
            telemetry.addData("Gp1 B", "Servo PWR -1");
            telemetry.addData("Gp1 Y", "Servo POS 0");
            telemetry.addData("POS", drive.clwLeft.instance.getPosition());
            telemetry.update();
        }
    }
}
