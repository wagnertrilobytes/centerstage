package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class PlaneLauncher implements Subsystem {
    Servo plane;
    @Override
    public void init(HardwareMap map) {
        plane = map.get(Servo.class, "plane");

        plane.setPosition(0.7);
    }

    public void sendPlane() {
        plane.setPosition(-0.7);
    }
    public void takePlaneBack() {
        plane.setPosition(0.7);
    }

    @Override
    public void run(Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry) {
        telemetry.addData("Plane Position", plane.getPosition());
    }
}
