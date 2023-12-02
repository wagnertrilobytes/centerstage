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
            telemetry.addLine("Motors: Drive");
            telemetry.addData("Front Left", robot.frontLeft.getCurrentPosition());
            telemetry.addData("Front Right", robot.frontRight.getCurrentPosition());
            telemetry.addData("Back Left", robot.backLeft.getCurrentPosition());
            telemetry.addData("Back Right", robot.backRight.getCurrentPosition());
            telemetry.addLine("Motors: Other");
            telemetry.addData("Slide Left", robot.slideLeft.getCurrentPosition());
            telemetry.addData("Slide Right", robot.slideRight.getCurrentPosition());
            telemetry.addData("Intake", robot.intake.getCurrentPosition());
            telemetry.addLine("Servos");
            telemetry.addData("Plane", "Not measurable sadly");
            telemetry.addData("Hook", robot.hook.getPosition());
            telemetry.addLine("Arrays");
            telemetry.addData("Total Motors", robot.motors.length);
            telemetry.addData("Total Drive Motors", robot.driveMotors.length);
            telemetry.update();
        }
    }
}
