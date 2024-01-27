package org.firstinspires.ftc.teamcode.opmodes.auto;

import android.icu.text.RelativeDateTimeFormatter;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

@Autonomous(name = "Motor Spin")
@Disabled
public class MotorSpin extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive robot = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        robot.clawLeft.turnToAngle(10);
        robot.clawRight.turnToAngle(10);

        while (opModeIsActive() && !isStopRequested()) {
            setMotor(robot.frontLeft);
            setMotor(robot.frontRight);
            setMotor(robot.backLeft);
            setMotor(robot.backRight);
            setMotor(robot.intake);
//            for(DcMotorEx motor : robot.motors) {
//                setMotor(motor);
//            }
        }
    }
    public void setMotor(DcMotorEx motor) {
        telemetry.addData("MOVING", motor.toString());
        telemetry.update();
        motor.setTargetPosition(motor.getCurrentPosition() + 500);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(1);
        sleep(250);
        motor.setPower(0);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sleep(1000);
    }
}
