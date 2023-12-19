package org.firstinspires.ftc.teamcode.helpers;

import com.qualcomm.robotcore.hardware.Servo;

public class PosCRServo {
    public static double powerMult = 1;
    public double lastPower = 1.0;
    Servo instance;
    public PosCRServo(Servo instance) {
        this.instance = instance;
    }
    public void zero() {
        this.instance.setPosition(0);
    }
    public void setPower(double power) {
        lastPower = power;
        this.instance.setPosition(this.instance.getPosition() * (lastPower * powerMult));
    }

    public static void setPowerMult(double powerMult) {
        PosCRServo.powerMult = powerMult;
    }

    public double getLastPower() {
        return lastPower;
    }

    public Servo getServo() {
        return this.instance;
    }
}
