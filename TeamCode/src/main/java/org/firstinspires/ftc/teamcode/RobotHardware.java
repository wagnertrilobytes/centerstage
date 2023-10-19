package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;

public class RobotHardware {
    /* Public OpMode members. */
    public RobotHardware robot = this;
    public DcMotor frontLeft = null;
    public DcMotor frontRight = null;
    public DcMotor backLeft = null;
    public DcMotor backRight = null;
    // public CRServo claw = null;
    // public Servo clawR = null;
    public DcMotor slideLeft = null;
    public DcMotor slideRight = null;
    public DcMotor plane = null;
    public DcMotor intake = null;
    public CRServo drop = null;
    public ArrayList<DcMotor> motors;
    public ArrayList<CRServo> servos;
    HardwareMap hwMap = null;
    // public BNO055IMU imu;
    /* Initialize standard hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to hardware map
        this.robot = this;
        hwMap = ahwMap;
        // Define and Initialize Motors
        // Wheels
        frontLeft = hwMap.get(DcMotor.class, "frontLeft");
        frontRight = hwMap.get(DcMotor.class, "frontRight");
        backLeft = hwMap.get(DcMotor.class, "backLeft");
        backRight = hwMap.get(DcMotor.class, "backRight");
        // The slides
        slideLeft = hwMap.get(DcMotor.class, "slideLeft");
        slideRight = hwMap.get(DcMotor.class, "slideRight");
        // The extra stuffs
        plane = hwMap.get(DcMotor.class, "plane");
        intake = hwMap.get(DcMotor.class, "intake");
        // We should probably make call this "drop" in the hardware map
        drop = hwMap.get(CRServo. class, "claw");
        motors.addAll(Arrays.asList(frontLeft, frontRight, backLeft, backRight, slideLeft, slideRight, plane, intake));
        servos.add(drop);

/*
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
*/

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        slideLeft.setDirection(DcMotor.Direction.FORWARD);
        slideRight.setDirection(DcMotor.Direction.FORWARD);

        plane.setDirection(DcMotor.Direction.REVERSE);

        // Set all motors to zero power, run with encoders, and ZPB to brake
        motors.forEach((e) -> {
            e.setPower(0);
            e.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            e.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        });
    }

    public void driveRobot(double Speed, double Turn, double Strafe) {
        // I did not take this from previous code..

        // Combine drive and turn for blended motion.
        double numFl = 0.45*Range.clip((+Speed - Turn - Strafe), -1, +1);
        double numFr = 0.45*Range.clip((+Speed + Turn + Strafe), -1, +1);
        double numBl = 0.35*Range.clip((+Speed + Turn - Strafe), -1, +1);
        double numBr = 0.45*Range.clip((+Speed - Turn + Strafe), -1, +1);

        // Scale the values so neither exceed +/- 1.0

        // Use existing function to drive both wheels.
        this.setAllPowerSpec(numFl, numFr, numBl, numBr);
    }

    public void setAllPowerSpec(double fl, double fr, double bl, double br) {
        this.frontLeft.setPower(fl);
        this.frontRight.setPower(fr);
        this.backLeft.setPower(bl);
        this.backRight.setPower(br);
    }
}

