package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.roadrunner.drive.YAMLPathOpMode;

@Disabled
public class ParsedAidenSplineYAML extends YAMLPathOpMode {
    public void onSetup() {
        this.setYamlFile("AidenSplineTestToo.yaml");
    }
}
