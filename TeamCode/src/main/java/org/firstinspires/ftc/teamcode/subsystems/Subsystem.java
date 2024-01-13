package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

public interface Subsystem {
    void init(SampleMecanumDrive map);
    void run(Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry);
}
