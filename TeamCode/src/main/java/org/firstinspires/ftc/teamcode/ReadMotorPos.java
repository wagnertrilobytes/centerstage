package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotHardware;

@TeleOp(name = "ReadMotorPos", group = "ReadVal")
public class ReadMotorPos extends LinearOpMode {
RobotHardware robot = new RobotHardware();
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("fl", robot.frontLeft.getCurrentPosition());
            telemetry.addData("fr", robot.frontRight.getCurrentPosition());
            telemetry.addData("bl", robot.backLeft.getCurrentPosition());
            telemetry.addData("br", robot.backRight.getCurrentPosition());
            telemetry.addData("sl", robot.slideLeft.getCurrentPosition());
            telemetry.addData("sr", robot.slideRight.getCurrentPosition());
            telemetry.addData("in", robot.intake.getCurrentPosition());
            telemetry.update();
        }
    }
}
