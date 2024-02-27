package org.firstinspires.ftc.teamcode.opmodes.tuners;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.SpicyBucket;

@TeleOp(name = "Servo POSITIONAL Tuner")
@Disabled
public class ServoPosTuner extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive robot = new SampleMecanumDrive(hardwareMap);
        SpicyBucket bucket = new SpicyBucket();
        bucket.init(hardwareMap);
        double angle = bucket.getCurrentBucketArmPos();
        waitForStart();
        while(opModeIsActive() && !isStopRequested()) {
        boolean abc = true;
            if(abc == true) {
                while (gamepad1.a) {
                    abc = false;
                }
                if (abc == false) {
                    angle += 0.1;
                }
            }

            boolean def = true;
            if(def == true) {
                while (gamepad1.b) {
                    def = false;
                }
                if (def == false) {
                    angle -= 0.1;
                }
            }
            bucket.setBucketArmPos(angle);
            telemetry.addData("barm", angle);
            telemetry.update();
        }
    }
}