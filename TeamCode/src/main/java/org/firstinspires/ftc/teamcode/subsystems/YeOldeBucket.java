package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.helpers.ServoToo;

public class YeOldeBucket implements Subsystem{
    Servo clawLeftServo;
    Servo clawRightServo;
    ServoToo clawLeft;
    ServoToo clawRight;
    @Override
    public void init(HardwareMap map) {
        clawLeftServo = map.get(Servo.class, "clawLeft");
        clawRightServo = map.get(Servo.class, "clawRight");

        clawLeft = new ServoToo(clawLeftServo, 0, 290, AngleUnit.DEGREES);
        clawRight = new ServoToo(clawRightServo, 0, 290, AngleUnit.DEGREES);

        clawRight.setInverted(true);
    }

    public double minArmAngle() {
        return clawLeft.min;
    }
    public double maxArmAngle() {
        return clawLeft.max;
    }
    public void turnTo(double angle) {
        clawLeft.turnToAngle(angle);
        clawRight.turnToAngle(angle);
    }

    @Override
    public void run(Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry) {
        telemetry.addData("CLeft Angle", clawLeft.getAngle());
        telemetry.addData("CRight Angle", clawRight.getAngle());
    }
}
