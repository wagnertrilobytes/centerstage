package org.firstinspires.ftc.teamcode.oldtest;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Motor ABXY Test", group="Old Tests")
public class newtest extends LinearOpMode {

    GameHardware robot = new GameHardware();
    @Override
    public void runOpMode() {
        robot.init(hardwareMap, this);

        waitForStart();
        while (opModeIsActive()) {
            if (gamepad2.a) {
                robot.LeftBack.setPower(1);
            } else {
                robot.LeftBack.setPower(0);
            }
            if (gamepad2.b) {
                robot.LeftFront.setPower(1);
            } else {
                robot.LeftFront.setPower(0);
            }

            if(gamepad2.x) {
                robot.RightBack.setPower(1);
            } else {
                robot.RightBack.setPower(0);
            }

            if(gamepad2.y) {
                robot.RightFront.setPower(1);
            } else {
                robot.RightFront.setPower(0);
            }
        }
    }
}
