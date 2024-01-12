package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

@TeleOp(name="servoreset")
public class ServoReset extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        waitForStart();
        while(opModeIsActive() && !isStopRequested()) {
            if (gamepad1.a) {
                drive.clwLeft.setPosition(0);
                drive.clwRight.setPosition(0);
            }
        }
    }
}
