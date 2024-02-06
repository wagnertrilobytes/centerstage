package org.firstinspires.ftc.teamcode.opmodes.auto.examples;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.helpers.Storage;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

@Autonomous(name = "Example RR Auto", group = "RR Example")
@Disabled
public class ExampleRRAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive robot = new SampleMecanumDrive(hardwareMap);
        waitForStart();
        TrajectorySequence trajSec = robot.trajectorySequenceBuilder(new Pose2d(35,35,Math.toRadians(270)))
                .splineTo(new Vector2d(45, 35), Math.toRadians(360))
                .build();
        robot.followTrajectorySequence(trajSec);
        double clawSpeed = 0.75;
//        robot.slideLeft.setTargetPosition(Storage.MINIMUM_SLIDE_PLACE_L);
//        robot.slideRight.setTargetPosition(Storage.MINIMUM_SLIDE_PLACE_R);
//        robot.slideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.slideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        while (robot.slideLeft.getCurrentPosition() != Storage.MINIMUM_SLIDE_PLACE_L ||
//            robot.slideRight.getCurrentPosition() != Storage.MINIMUM_SLIDE_PLACE_R) {}
//        robot.slideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.slideLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.slideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.slideRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ElapsedTime a = new ElapsedTime();
        while (a.seconds() != 1) {
//            robot.clawLeft.turnToAngle(robot.clawLeft.max);
//            robot.clawRight.turnToAngle(robot.clawRight.max);
        }
        sleep(750);

        robot.followTrajectorySequence(robot.trajectorySequenceBuilder(trajSec.end())
                .setReversed(true)
                .build());
    }
}
