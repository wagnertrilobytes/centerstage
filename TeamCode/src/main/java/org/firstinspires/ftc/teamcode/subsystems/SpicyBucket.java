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
    Servo armServo;
    ServoToo arm;
    @Override
    public void init(HardwareMap map) {
        wheel = map.get(CRServo.class, "clawRight");
        armServo = map.get(Servo.class, "clawLeft");
        arm = new ServoToo(armServo, 0, 290, AngleUnit.DEGREES);

        wheel.setPower(0);
    }
    public void dropOnePixel(LinearOpMode opMode) {
        wheel.setPower(-1);
        opMode.sleep(925);
        wheel.setPower(0);
    }
    public void setArmAngle(double angle) {
        arm.turnToAngle(angle);
    }

    public double maxArmAngle() {return arm.max;}
    public double minArmAngle() {return arm.min;}

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
        telemetry.addData("Arm Angle", arm.getAngle());
        telemetry.addData("Wheel Power", wheel.getPower());
    }
}
