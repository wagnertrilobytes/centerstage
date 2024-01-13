package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

@Autonomous(name = "Camera Subsystem Test")
public class CameraScanSubsystemTest extends OpMode {
    SampleMecanumDrive rob = new SampleMecanumDrive(hardwareMap);
    CameraScanSubsystem sub = new CameraScanSubsystem();
    @Override
    public void init() {
        sub.init(rob);
    }

    @Override
    public void loop() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addData("Hi", "yes");
        sub.run(gamepad1, gamepad2, telemetry);
        telemetry.update();
    }
}
