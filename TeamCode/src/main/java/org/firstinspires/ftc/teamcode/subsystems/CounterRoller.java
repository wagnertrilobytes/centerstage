package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class CounterRoller implements Subsystem{
    CRServo roller;
    @Override
    public void init(HardwareMap map) {
        roller = map.get(CRServo.class, "roller");

        roller.setPower(0);
        roller.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void spinForward() {
        roller.setPower(1);
    }

    public void spinBackward() {
        roller.setPower(-1);
    }

    public void stop() {
        roller.setPower(0);
    }

    public double getPower() {
        return roller.getPower();
    }

    @Override
    public void run(Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry) {

    }
}
