package org.firstinspires.ftc.teamcode.opmodes.tuners;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Slides;
import org.firstinspires.ftc.teamcode.subsystems.SpicyBucket;

@Autonomous(name = "Pos Tuner")
@Disabled
public class PosTuner extends LinearOpMode {
    int mode;
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive robot = new SampleMecanumDrive(hardwareMap);
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        SpicyBucket bucket = new SpicyBucket();
        Slides slides = new Slides();
        Intake intake = new Intake();
        intake.init(hardwareMap);
        slides.init(hardwareMap);
        bucket.init(hardwareMap);
        waitForStart();
        while(opModeIsActive() && !isStopRequested()) {
            telemetry.addData("Drive Motors FrontLeft", fmt(robot.frontLeft));
            telemetry.addData("Drive Motors FrontRight", fmt(robot.backRight));
            telemetry.addData("Drive Motors BackLeft", fmt(robot.backLeft));
            telemetry.addData("Drive Motors BackRight", fmt(robot.backRight));

            telemetry.addLine();
            telemetry.addLine();

            telemetry.addData("Bucket BucketArm Pos", bucket.getCurrentBucketArmPos());
            telemetry.addData("Bucket BucketArm Angle", bucket.getCurrentBucketArmAngle());
            telemetry.addData("Bucket SlideArm Angle", bucket.getCurrentSlideArmAngle());

            telemetry.addLine();
            telemetry.addLine();

            telemetry.addData("Slides SlideLeft Pos", slides.getLeftPos());
            telemetry.addData("Slides SlideRight Pos", slides.getRightPos());

            telemetry.update();
        }
    }

    public String fmt(DcMotorEx mtr) {
        return mtr.getCurrentPosition() + "@" + mtr.getPower() + ":" + mtr.getPortNumber();
    }
}
