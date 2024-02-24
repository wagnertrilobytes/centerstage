package org.firstinspires.ftc.teamcode.opmodes.auto.encoderbased;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

@Autonomous(name = "Simple Auto", group="Fallback")
public class SimpleAutoRR extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive robot = new SampleMecanumDrive(hardwareMap);

        Intake intake = new Intake();
        intake.init(hardwareMap);

        waitForStart();

        robot.followTrajectorySequence(
                robot.trajectorySequenceBuilder(new Pose2d(0,0,0))
                        .forward(55)
                        .build()
        );

        ElapsedTime timer = new ElapsedTime();
        while(timer.seconds() > 2) {
            intake.setPower(1, -1);
        }

        intake.stop();
    }
}
