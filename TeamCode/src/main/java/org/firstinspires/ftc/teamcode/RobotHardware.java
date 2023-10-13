package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class RobotHardware {
    /* Public OpMode members. */
    public DcMotor frontLeft = null;
    public DcMotor frontRight = null;
    public DcMotor backLeft = null;
    public DcMotor backRight = null;
    //public CRServo claw = null;
    //public Servo clawR = null;
    public DcMotor slideLeft = null;
    public DcMotor slideRight = null;
    //   public DcMotor armB = null;
    public DcMotor plane = null;
    public DcMotor intake = null;
    public CRServo drop = null;
    /* local OpMode members. */

    HardwareMap hwMap = null;
   // BNO055IMU imu;

    private ElapsedTime period = new ElapsedTime();
    /* Constructor */
    public void test(){
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
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

        plane = hwMap.get(DcMotor.class, "plane");
        intake = hwMap.get(DcMotor.class, "intake");

        drop = hwMap.get(CRServo. class, "claw");

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
        frontLeft.setDirection (DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        slideLeft.setDirection(DcMotor.Direction.FORWARD);
        slideRight.setDirection(DcMotor.Direction.FORWARD );

        plane.setDirection(DcMotor.Direction.REVERSE);
        //armB.setDirection(DcMotor.Direction.REVERSE);

        // Set all motors to zero power
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
        slideLeft.setPower(0);
        slideRight.setPower(0);
        plane.setPower(0);
        intake.setPower(0);
        drop.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        plane.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //  armB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
       // claw = hwMap.get(CRServo.class, "claw");
        //clawR = hwMap.get(Servo.class, "clawR");


        //Brake Function
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        plane.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //    armB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


    }

    public void driveRobot(double Drive, double Turn, double strafe) {
        // I did not take this from previous code..

        // Combine drive and turn for blended motion.
        double le = Drive + Turn + strafe;
        double ri = Drive - Turn - strafe;
        double lb = Drive + Turn - strafe;
        double rb = Drive - Turn + strafe;

        // Scale the values so neither exceed +/- 1.0

        // Use existing function to drive both wheels.
        this.setAllPowerSpec(le, ri ,lb ,rb);
    }

    public void setAllPower(double power) {
        this.frontLeft.setPower(power);
        this.frontRight.setPower(power);
        this.backLeft.setPower(power);
        this.backRight.setPower(power);
    }

    public void setAllPowerSpec(double fl, double fr, double bl, double br) {
        this.frontLeft.setPower(fl);
        this.frontRight.setPower(fr);
        this.backLeft.setPower(bl);
        this.backRight.setPower(br);
    }
}

