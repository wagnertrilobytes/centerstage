package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Servo Reading", group = "Test")
public class ServoTest extends LinearOpMode {
    RobotHardware robot = new RobotHardware();
    @Override
    public void runOpMode() {
        robot.init(hardwareMap, this, false);

        waitForStart();
        while (opModeIsActive()) {
            if (gamepad2.x) {
                telemetry.addData("Hook Pos", robot.hook.getPosition());
                telemetry.addData("Plane Pos", robot.plane.getPosition());
                telemetry.addData("Drop Power", robot.drop.getPower());
                telemetry.update();
            }
            telemetry.addData("Hook Pos", robot.hook.getPosition());
            telemetry.addData("Plane Pos", robot.plane.getPosition());
            telemetry.addData("Drop Power", robot.drop.getPower());
            telemetry.update();
        }
    }
}
