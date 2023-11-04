package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@Config
@TeleOp
public class PIDTestingOpMode extends OpMode {
    private PIDController controller;
    public static double p = 0.005,
                        i = 0,
                        d = 0.0; // d = dampener (dampens arm movement and is scary). ignore i
    public static double f = .002;  // prevents arm from falling from gravity

    public static double rightFinches = 55;
    public static double rightBinches = 55;
    public static double leftFinches = 55;
    public static double leftBinches = 55;
    public RobotHardware robot = new RobotHardware();

    @Override
    public void init() {
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot.init(hardwareMap); // initializing hardware map

        robot.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {
        controller.setPID(p, i, d);

        double flPos = robot.frontLeft.getCurrentPosition();
        double frPos = robot.frontRight.getCurrentPosition();
        double blPos = robot.backLeft.getCurrentPosition();
        double brPos = robot.backRight.getCurrentPosition();

        double flPid = controller.calculate(flPos, leftFinches);
        double frPid = controller.calculate(frPos, rightFinches);
        double blPid = controller.calculate(blPos, leftBinches);
        double brPid = controller.calculate(brPos, rightBinches);

        double flPower = flPid + f;
        double frPower = frPid + f;
        double blPower = blPid + f;
        double brPower = brPid + f;

        robot.frontLeft.setPower(flPower);
        robot.frontRight.setPower(frPower);
        robot.backLeft.setPower(blPower);
        robot.backRight.setPower(brPower);

        telemetry.addData("fl pos", flPos);
        telemetry.addData("fr pos", frPos);
        telemetry.addData("bl pos", blPos);
        telemetry.addData("br pos", brPos);
        telemetry.addLine();
        telemetry.addData("fl target", leftFinches);
        telemetry.addData("fr target", rightFinches);
        telemetry.addData("bl target", leftBinches);
        telemetry.addData("br target", rightBinches);
        telemetry.update();
    }
}


