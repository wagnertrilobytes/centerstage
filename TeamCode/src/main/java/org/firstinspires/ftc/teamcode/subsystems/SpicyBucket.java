package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.helpers.ServoToo;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

public class SpicyBucket implements Subsystem {
    CRServo wheel;
    Servo slideArmLServo;
    Servo slideArmRServo;
    ServoToo slideArmL;
    ServoToo slideArmR;
    Servo bucketArmLServo;
    Servo bucketArmRServo;
    ServoToo bucketArmL;
    ServoToo bucketArmR;
    @Override
    public void init(HardwareMap map) {
        wheel = map.get(CRServo.class, "wheel"); // p5

        slideArmLServo = map.get(Servo.class, "slideArmLeft"); // p1
        slideArmRServo = map.get(Servo.class, "slideArmRight"); // p0

        slideArmL = new ServoToo(slideArmLServo, 0, 290, AngleUnit.DEGREES);
        slideArmR = new ServoToo(slideArmRServo, 0, 290, AngleUnit.DEGREES);

        bucketArmLServo = map.get(Servo.class, "armBucketL"); // p2
        bucketArmRServo = map.get(Servo.class, "armBucketR"); // p4

        bucketArmL = new ServoToo(bucketArmLServo, -90, 180, AngleUnit.DEGREES);
        bucketArmR = new ServoToo(bucketArmRServo, -90, 180, AngleUnit.DEGREES);

        slideArmR.setInverted(true);
        bucketArmR.setInverted(true);

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
    public void setBucketArmAngle(double angle) {
        bucketArmL.turnToAngle(angle);
        bucketArmR.turnToAngle(angle);
    }
    public void setBucketArmPos(double pos) {
        bucketArmL.setPosition(pos);
        bucketArmR.setPosition(pos);
    }

    public double getCurrentBucketArmAngle() {
        return bucketArmL.getAngle();
    }
    public double getCurrentBucketArmPos() {
        return bucketArmL.getPosition();
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

    public double minBucketArmAngle() {
        return bucketArmL.min;
    }
    public double maxBucketArmAngle() {
        return bucketArmL.max;
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
        telemetry.addData("BucketLeft Arm Angle", bucketArmL.getAngle());
        telemetry.addData("BucketRight Arm Angle", bucketArmR.getAngle());
        telemetry.addData("Wheel Power", wheel.getPower());
    }
}
