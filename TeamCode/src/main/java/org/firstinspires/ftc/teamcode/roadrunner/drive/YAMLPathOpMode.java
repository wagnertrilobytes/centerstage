package org.firstinspires.ftc.teamcode.roadrunner.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.yaml.snakeyaml.Yaml;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.InputStream;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Disabled
@Autonomous(group = "drive")
public class YAMLPathOpMode extends LinearOpMode {
    public String yamlFile = "None.yaml";
    public void onSetup() {}
    public void setYamlFile(String file) {
        this.yamlFile = file;
    }
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        this.onSetup();
        waitForStart();

        if (isStopRequested()) return;

        try {
            // THIS IS A WIP!!!
            InputStream inputStream = new FileInputStream(new File(this.yamlFile));
            Yaml yaml = new Yaml();
        } catch (FileNotFoundException e) {
            telemetry.addData("Oopsies","Could not find the file specified.");
            stop();
        }
    }
}
