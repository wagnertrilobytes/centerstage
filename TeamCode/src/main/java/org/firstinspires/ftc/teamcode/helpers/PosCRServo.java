package org.firstinspires.ftc.teamcode.helpers;

import com.qualcomm.robotcore.hardware.Servo;

public class PosCRServo {
    public static double powerMult = 1;
    public double lastPower = 1.0;
    Servo instance;
    public PosCRServo(Servo instance) {
        this.instance = instance;
        this.instance.setPosition(0.1);
    }
    public void zero() {
        this.instance.setPosition(0.1);
    }
    public void setPower(double power) {
        lastPower = power;
        if(this.instance.getPosition() > 0) this.instance.setPosition(0.1);
        this.instance.setPosition(this.instance.getPosition() *power);
    }

    public static void setPowerMult(double powerMult) {
        PosCRServo.powerMult = powerMult;
    }

    public double getLastPower() {
        return lastPower;
    }
    public void loop() {
        if (this.instance.getPosition() == 0.0) {
            this.instance.setPosition(0.1);
        }
    }

    public Servo getServo() {
        return this.instance;
    }
}
