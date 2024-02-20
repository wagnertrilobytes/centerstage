package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.helpers.ServoToo;

public class SpicyBucketCR implements Subsystem {
    public CRServo wheel;
    public CRServo bucketArmL;
    public CRServo bucketArmR;
    public CRServo slideArmL;
    public CRServo slideArmR;
    @Override
    public void init(HardwareMap map) {
        wheel = map.get(CRServo.class, "wheel"); // p5

        slideArmL = map.get(CRServo.class, "slideArmLeft"); // p1
        slideArmR = map.get(CRServo.class, "slideArmRight"); // p0

        bucketArmL = map.get(CRServo.class, "armBucketL"); // p2
        bucketArmR = map.get(CRServo.class, "armBucketR"); // p4


        slideArmR.setDirection(DcMotorSimple.Direction.REVERSE);
        bucketArmR.setDirection(DcMotorSimple.Direction.REVERSE);

        bucketArmL.setPower(0);
        bucketArmR.setPower(0);

        slideArmL.setPower(0);
        slideArmR.setPower(0);

        wheel.setPower(0);
    }
    public void dropOnePixel() {
        ElapsedTime timer = new ElapsedTime();
        while (timer.milliseconds() < 240) {
            wheel.setPower(-1);
        }
        wheel.setPower(0);
    }

    public void setSlideArmPower(double pwr) {
        slideArmL.setPower(pwr);
        slideArmR.setPower(pwr);
    }
    public void setBucketArmPower(double pwr) {
        bucketArmL.setPower(pwr);
        bucketArmR.setPower(pwr);
    }

    public double maxArmAngle() {return 360;}
    public double minArmAngle() {return 360;}

    public double getCurrentBucketArmPos() {
        return bucketArmL.getPower();
    }

    public void takeIn() {
        wheel.setPower(1);
    }

    public void takeOut() {
        wheel.setPower(-1);
    }

    public void stop() {
        wheel.setPower(0);
    }
    public void setSlideArmAngle(double d) {}

    @Override
    public void run(Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry) {
        telemetry.addData("SlideLeft Arm Angle", slideArmL.getPower());
        telemetry.addData("SlideRight Arm Angle", slideArmR.getPower());
        telemetry.addData("BucketLeft Arm Angle", bucketArmL.getPower());
        telemetry.addData("BucketRight Arm Angle", bucketArmR.getPower());
        telemetry.addData("Wheel Power", wheel.getPower());
    }
}
