package org.firstinspires.ftc.teamcode.opmodes.test;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.helpers.Storage;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

@Autonomous(name = "Async FiniteStateMachine")
public class AsyncFSMTest extends LinearOpMode {
    enum State {
        HELLO_WORLD,
        IDLE
    }

    interface Poses {
        interface Backstage {
            Pose2d RED = new Pose2d(35, -60, Math.toRadians(90));
            Pose2d BLUE = new Pose2d(15, 60, Math.toRadians(270));
        }
    }

    State currentState = State.IDLE;

    Pose2d startPose = Poses.Backstage.RED;
    //IMPORTANT: Change this if needed

    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Storage.currentPose = startPose;
        drive.setPoseEstimate(startPose);

        waitForStart();
        if(isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {

            switch(currentState) {
                case HELLO_WORLD:
                    telemetry.addData("HELLO", "WORLD");
                    break;
                case IDLE:
                    break;
            }

            drive.update();
            Pose2d poseEstimate = drive.getPoseEstimate();
            Storage.currentPose = poseEstimate;

            telemetry.addData("Side", startPose == Poses.Backstage.RED ? "Red" : "Blue");
            telemetry.addData("State", currentState);
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();
        }
    }
}
