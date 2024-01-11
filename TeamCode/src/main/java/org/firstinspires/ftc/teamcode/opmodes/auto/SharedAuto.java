package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

public class SharedAuto {
    public void slideUpAndScore(SampleMecanumDrive robot, LinearOpMode opMode) {
        /*  THIS IS EXPERIMENTAL  */
        double clawSpeed = 0.75;
        robot.slideLeft.setPower(-0.75);
        robot.slideRight.setPower(0.75);
        opMode.sleep(750);
        robot.slideLeft.setPower(0);
        robot.slideRight.setPower(0);
        opMode.sleep(250);
        ElapsedTime a = new ElapsedTime();
        while (a.seconds() != 1) {
            robot.clawLeft.turnToAngle(robot.clawLeft.max);
            robot.clawRight.turnToAngle(robot.clawRight.max);
        }
    }
}
