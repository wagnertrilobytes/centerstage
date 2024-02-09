package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Slides implements Subsystem {
    DcMotorEx slideLeft, slideRight;
    @Override
    public void init(HardwareMap map) {
        slideLeft = map.get(DcMotorEx.class, "slideLeft");
        slideRight = map.get(DcMotorEx.class, "slideRight");

        slideLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        slideLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void moveTo(int position) {
        slideLeft.setTargetPosition(position);
        slideRight.setTargetPosition(position);

        slideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void stop() {
        setPower(0);
    }

    public void setPower(double power) {
        slideLeft.setPower(power);
        slideRight.setPower(power);
    }

    public double getLeftPos() {
        return slideLeft.getCurrentPosition();
    }

    public double getRightPos() {
        return slideRight.getCurrentPosition();
    }

    @Override
    public void run(Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry) {
        telemetry.addData("Slide Left (Pos@Pow,Port)", "(" + slideLeft.getCurrentPosition() + "@" + slideLeft.getPower() + "," + slideLeft.getPortNumber() +")");
        telemetry.addData("Slide Right (Pos@Pow,Port)", "(" + slideRight.getCurrentPosition() + "@" + slideRight.getPower() + "," + slideRight.getPortNumber() +")");
    }
}
