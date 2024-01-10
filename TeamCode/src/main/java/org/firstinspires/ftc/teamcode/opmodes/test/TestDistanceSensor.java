package org.firstinspires.ftc.teamcode.opmodes.test;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequenceBuilder;

//@Config
@Autonomous(name = "Testing: DISTANCE SNESOR!!!!!!!!!!!!")
@Disabled
public class TestDistanceSensor extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        waitForStart();
            Rev2mDistanceSensor dist = hardwareMap.get(Rev2mDistanceSensor.class, "dist");
            drive.setPoseEstimate(new Pose2d(0,0,0));
        while(opModeIsActive() && !isStopRequested()) {
            telemetry.addData("DIST (inches)", dist.getDistance(DistanceUnit.INCH));
            if (gamepad1.a) {
                TrajectorySequenceBuilder trajSecb = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .forward(dist.getDistance(DistanceUnit.INCH) - 0.5);
                TrajectorySequence trajSec = trajSecb.build();
                drive.followTrajectorySequence(trajSec);
            }
            if (gamepad2.b) {
                double ERR = 1.123;
                telemetry.addData("Checking if", "distance in inches is 5.. (with err tolerance of " + ERR+")");
                if (dist.getDistance(DistanceUnit.INCH) > 5+ERR || dist.getDistance(DistanceUnit.INCH) < 5+ERR) {
                    telemetry.addData("Its not :(", "Inching up (by 0.6in) until it is.");
                    TrajectorySequenceBuilder trajSecb = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                            .forward(0.6);
                    TrajectorySequence trajSec = trajSecb.build();
                    drive.followTrajectorySequence(trajSec);
                } else {
                    telemetry.addData("It is!", "Yippee");
                }
            }
            telemetry.update();
        }
    }
}
