package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

public class RobotHardware {
    /* Public OpMode members. */
    private final ElapsedTime runtime = new ElapsedTime();
    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;
    public RobotHardware robot = this;
    public DcMotor frontLeft = null;
    public DcMotor frontRight = null;
    public DcMotor backLeft = null;
    public DcMotor backRight = null;
    // public CRServo claw = null;
    // public Servo clawR = null;
    public DcMotor slideLeft = null;
    public DcMotor slideRight = null;
    public DcMotor intake = null;
    public CRServo drop = null;
    public Servo plane = null;
    public Servo hook = null;
    public WebcamName camera = null;
    public int tileSizeInches = 24;
//    public BNO055IMU imu = null;
    public DcMotor[] motors;
    public CRServo[] cservos;
    public Servo[] servos;
    public DcMotor[] driveMotors;

    public LinearOpMode _opMode;
    public HardwareMap _hwMap;
    public Telemetry telemetry;
    public boolean auto;

    /* Math */
    public double SLIDE_SPEED = 0.15;
    HardwareMap hwMap = null;
    // public BNO055IMU imu;
    /* Initialize standard hardware interfaces */
    public void init(HardwareMap ahwMap, LinearOpMode opMode, boolean initAuto) {
        // Save reference to hardware map
        this.robot = this;
        this._opMode = opMode;
        this._hwMap = ahwMap;
        hwMap = ahwMap;
        if (initAuto) this.auto = true;

        this.telemetry = this._opMode.telemetry;
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
        intake = hwMap.get(DcMotor.class, "intake");
        // We should probably make call this "drop" in the hardware map
        drop = hwMap.get(CRServo.class, "claw");
        plane = hwMap.get(Servo.class, "plane");
        hook = hwMap.get(Servo.class, "hook");
//        camera = hwMap.get(WebcamName.class, "camera");
        motors = new DcMotor[]{frontLeft, frontRight, backLeft, backRight, slideLeft, slideRight, intake};
        cservos = new CRServo[]{ drop };
        servos = new Servo[] { plane, hook };
        driveMotors = new DcMotor[]{frontLeft,frontRight, backLeft, backRight};

//        servos.add(drop);
//        imu = hwMap.get(BNO055IMU.class, "imu");

//        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
//        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
//        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
//        parameters.loggingEnabled = true;
//        parameters.loggingTag = "IMU";
//        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

//        imu.initialize(parameters);

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        slideLeft.setDirection(DcMotor.Direction.FORWARD);
        slideRight.setDirection(DcMotor.Direction.FORWARD);

        // Set all motors to zero power, run with encoders, and ZPB to brake
        for (DcMotor e : motors)
        {
            e.setPower(0);
            e.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            e.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        this.telemetry.addData("Fabian Bafoonery", "Fabian Bafoonery");
        this.telemetry.update();
    }

    public int tiles(int amount) {
        return amount * this.tileSizeInches;
    }

    public void addPowerOnButtonPress(boolean button, DcMotor motor, double pressPower, double releasePower) {
        if (button) { motor.setPower(pressPower); } else { motor.setPower(releasePower); }
    }
    public void addPowerOnButtonPress(boolean button, CRServo servo, double pressPower, double releasePower) {
        if (button) { servo.setPower(pressPower); } else { servo.setPower(releasePower); }
    }

    public void driveRobot(double Drive, double Turn, double strafe) {
        // Combine drive and turn for blended motion.
//        double numFl = 0.45*Range.clip((+Speed - Turn - Strafe), -1, +1);
//        double numFr = 0.45*Range.clip((+Speed + Turn + Strafe), -1, +1);
//        double numBl = 0.45*Range.clip((+Speed + Turn - Strafe), -1, +1);
//        double numBr = 0.45*Range.clip((+Speed - Turn + Strafe), -1, +1);

        double fl = Drive + Turn + strafe;
        double fr = Drive - Turn - strafe;
        double bl = Drive + Turn - strafe;
        double br = Drive - Turn + strafe;

        // Scale the values so neither exceed +/- 1.0

        // Use existing function to drive both wheels.
//        this.setAllPowerSpec(numFl, numFr, numBl, numBr);
        this.setAllPowerSpec(fl, fr, bl, br);
    }

    public void setAllPowerSpec(double fl, double fr, double bl, double br) {
        this.frontLeft.setPower(fl);
        this.frontRight.setPower(fr);
        this.backLeft.setPower(bl);
        this.backRight.setPower(br);
    }

    public void toggleButton(CRServo thing, double pwr, boolean button) {
        boolean tempToggleBool = true;
        if(tempToggleBool) {
            while (button) {
                thing.setPower(pwr);
                tempToggleBool = false;
            }
            if (tempToggleBool == false) thing.setPower(0);
        }
    }

    public void toggleButton(DcMotor thing, double pwr, boolean button) {

    }
}

