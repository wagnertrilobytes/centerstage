package org.firstinspires.ftc.teamcode.opmodes.tuners;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.SpicyBucket;

@Autonomous(name = "zero")
@Disabled
public class Zero extends LinearOpMode{
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive robot = new SampleMecanumDrive(hardwareMap);
        SpicyBucket bucket = new SpicyBucket();
        bucket.init(hardwareMap);
        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
//            bucket.setBucketArmPos(0);
            telemetry.addData("bucket rmpaorwoijgre", bucket.getCurrentBucketArmPos());
            telemetry.update();
        }
    }
}
