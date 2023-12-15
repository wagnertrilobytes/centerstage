package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.hardware.Servo;

public class TestServo {
    public static double powerMult = 0.10;
    Servo instance;
    public TestServo(Servo instance) {
        this.instance = instance;
    }

    public void zero() {
        this.instance.setPosition(0);
    }
    public void positive(double power) {
        this.instance.setPosition(this.instance.getPosition() + (power*powerMult));
    }

    public void negative(double power) {
        this.instance.setPosition(this.instance.getPosition() - (power*powerMult));
    }

    public Servo getServo() {
        return this.instance;
    }
}
