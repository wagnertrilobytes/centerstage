package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

@Config
public class SharedAuto {
    public static double clawSpeed = 0.75;
    public void slideUpAndScore(SampleMecanumDrive robot, LinearOpMode opMode) {
        /*  THIS IS EXPERIMENTAL  */
        robot.slideLeft.setPower(-0.75);
        robot.slideRight.setPower(0.75);
        opMode.sleep(1000);
        robot.slideLeft.setPower(0);
        robot.slideRight.setPower(0);
        opMode.sleep(250);
        robot.clawLeft.turnToAngle(robot.clawLeft.max - 15);
        robot.clawRight.turnToAngle(robot.clawRight.max - 15);
    }
}
