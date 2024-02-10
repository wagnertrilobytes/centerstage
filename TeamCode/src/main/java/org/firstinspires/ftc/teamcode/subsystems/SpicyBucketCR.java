package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.helpers.ServoToo;

public class SpicyBucketCR implements Subsystem {
    CRServo wheel;
    Servo slideArmLServo;
    Servo slideArmRServo;
    ServoToo slideArmL;
    ServoToo slideArmR;
    CRServo bucketArmL;
    CRServo bucketArmR;
    @Override
    public void init(HardwareMap map) {
        wheel = map.get(CRServo.class, "wheel"); // p5

        slideArmLServo = map.get(Servo.class, "slideArmLeft"); // p1
        slideArmRServo = map.get(Servo.class, "slideArmRight"); // p0

        slideArmL = new ServoToo(slideArmLServo, 0, 290, AngleUnit.DEGREES);
        slideArmR = new ServoToo(slideArmRServo, 0, 290, AngleUnit.DEGREES);

        bucketArmL = map.get(CRServo.class, "armBucketL"); // p2
        bucketArmR = map.get(CRServo.class, "armBucketR"); // p4


        slideArmR.setInverted(true);
        bucketArmR.setDirection(DcMotorSimple.Direction.REVERSE);

        bucketArmL.setPower(0);
        bucketArmR.setPower(0);

        wheel.setPower(0);
    }
    public void dropOnePixel(LinearOpMode opMode) {
        wheel.setPower(-1);
        opMode.sleep(925);
        wheel.setPower(0);
    }
    public void setSlideArmAngle(double angle) {
        slideArmL.turnToAngle(angle);
        slideArmR.turnToAngle(angle);
    }
    public void setBucketArmPower(double pwr) {
        bucketArmL.setPower(pwr);
        bucketArmR.setPower(pwr);
    }

    public double getCurrentBucketArmPos() {
        return bucketArmL.getPower();
    }
    public double getCurrentSlideArmAngle() {
        return slideArmL.getAngle();
    }

    public double maxArmAngle() {
        return slideArmL.max;
    }
    public double minArmAngle() {
        return slideArmL.min;
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

    @Override
    public void run(Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry) {
        telemetry.addData("SlideLeft Arm Angle", slideArmL.getAngle());
        telemetry.addData("SlideRight Arm Angle", slideArmR.getAngle());
        telemetry.addData("BucketLeft Arm Angle", bucketArmL.getPower());
        telemetry.addData("BucketRight Arm Angle", bucketArmR.getPower());
        telemetry.addData("Wheel Power", wheel.getPower());
    }
}
