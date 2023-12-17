package org.firstinspires.ftc.teamcode.roadrunner.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequenceBuilder;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive", name="Backstage Blue Right Spline Test")
@Disabled
public class NeelSplineTestThree extends LinearOpMode {
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

        // TRY 2
//        Pose2d startPos = new Pose2d(12, 63, Math.toRadians(270));
//        drive.setPoseEstimate(startPos);
//        drive.followTrajectorySequence(drive.trajectorySequenceBuilder(new Pose2d(0,0,0))
//                .splineTo(new Vector2d(startPos.getX(), 40), Math.toRadians(0))
//                .build());

        //TRY 3
//        drive.followTrajectorySequence(drive.trajectorySequenceBuilder(new Pose2d())
//                        .forward(24)
//                        .turn(Math.toRadians(0))
//                .build());
        // TRY 4, USING ROADRUNNER GUI
        Pose2d startPose = new Pose2d(14, 61, Math.toRadians(270));
        drive.setPoseEstimate(startPose);
        TrajectorySequenceBuilder stepOne = drive.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(14, 44))
                .lineTo(new Vector2d(5, 44))
                .splineTo(new Vector2d(5, 35),Math.toRadians(90));
        TrajectorySequence sob = stepOne.build();
        drive.followTrajectorySequence(sob);
        hardwareMap.get(DcMotor.class, "intake").setPower(0.4);
        sleep(250);
        hardwareMap.get(DcMotor.class, "intake").setPower(0);
        TrajectorySequenceBuilder st = drive.trajectorySequenceBuilder(sob.end())
                .lineTo(new Vector2d(25, 50))
                .lineTo(new Vector2d(46, 40));
        TrajectorySequence stb = st.build();
        drive.followTrajectorySequence(stb);
    }
}