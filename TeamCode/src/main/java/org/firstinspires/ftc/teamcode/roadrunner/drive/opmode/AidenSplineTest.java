package org.firstinspires.ftc.teamcode.roadrunner.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
public class AidenSplineTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

//        TrajectorySequence traj = drive.trajectorySequenceBuilder(new Pose2d())
//                .forward(26)
//                .turn(Math.toRadians(90))
//                .build();
//
//        drive.followTrajectorySequence(traj);
//        hardwareMap.get(DcMotor.class, "intake").setPower(0.4);
//        sleep(2500);
//        hardwareMap.get(DcMotor.class, "intake").setPower(0);
//        drive.followTrajectorySequence(drive.trajectorySequenceBuilder(traj.end())
//                .turn(Math.toRadians(-90))
//                .back(26)
//                .turn(Math.toRadians(90))
//                .forward(40)
//                .turn(Math.toRadians(-90))
//                .forward(24)
//                .build());
        drive.setPoseEstimate(new Pose2d(7, 61, 0));
        drive.followTrajectorySequence(drive.trajectorySequenceBuilder(new Pose2d(7, 61, 0))
                .splineTo(new Vector2d(7, 33), 3)
                .build());
    }
}
