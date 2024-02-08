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
    Servo slideArmServo;
    Servo bucketArmServo;
    ServoToo slideArm;
    ServoToo bucketArm;
    @Override
    public void init(HardwareMap map) {
        wheel = map.get(CRServo.class, "clawRight");

        slideArmServo = map.get(Servo.class, "clawLeft");
        slideArm = new ServoToo(slideArmServo, 0, 290, AngleUnit.DEGREES);

        bucketArmServo = map.get(Servo.class, "armBucket");
        bucketArm = new ServoToo(bucketArmServo, 0, 290, AngleUnit.DEGREES);

        wheel.setPower(0);
    }
    public void dropOnePixel(LinearOpMode opMode) {
        wheel.setPower(-1);
        opMode.sleep(925);
        wheel.setPower(0);
    }
    public void setSlideArmAngle(double angle) {
        slideArm.turnToAngle(angle);
    }
    public void setBucketArmAngle(double angle) {
        bucketArm.turnToAngle(angle);
    }

    public double maxArmAngle() {return slideArm.max;}
    public double minArmAngle() {return slideArm.min;}

    public double minBucketArmAngle() {return bucketArm.min;}
    public double maxBucketArmAngle() {return bucketArm.max;}

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
        telemetry.addData("Slide Arm Angle", slideArm.getAngle());
        telemetry.addData("Bucket Arm Angle", bucketArm.getAngle());
        telemetry.addData("Wheel Power", wheel.getPower());
    }
}
