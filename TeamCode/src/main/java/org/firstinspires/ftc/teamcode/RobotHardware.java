package org.firstinspires.ftc.teamcode;

// import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.vision.ColourMassDetectionProcessor;

@Disabled
public class RobotHardware {
    /* Public OpMode members. */
    public DcMotor frontLeft = null;
    public DcMotor frontRight = null;
    public DcMotor backLeft = null;
    public DcMotor backRight = null;
    public CRServo clawLeft = null;
    public CRServo clawRight = null;
    //public Servo clawR = null;
    public DcMotor slideLeft = null;
    public DcMotor slideRight = null;
    public DcMotor intake = null;
    public WebcamName camera = null;
    public Servo plane = null;

    public Servo hook = null;
    public DcMotor[] motors;
    public DcMotor[] driveMotors;
    public ColourMassDetectionProcessor.PropPositions lastPropPos = null;
    public int TILE_LEN = 24;
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
        frontLeft = hwMap.get(DcMotor.class, "frontLeft");
        frontRight = hwMap.get(DcMotor.class, "frontRight");
        backLeft = hwMap.get(DcMotor.class, "backLeft");
        backRight = hwMap.get(DcMotor.class, "backRight");
        slideLeft = hwMap.get(DcMotor.class, "slideLeft");
        camera = hwMap.get(WebcamName.class, "Webcam 1");
        slideRight = hwMap.get(DcMotor.class, "slideRight");
        intake = hwMap.get(DcMotor.class, "intake");
        clawLeft = hwMap.get(CRServo.class,"clawLeft");
        clawRight = hwMap.get(CRServo.class, "clawRight");
        plane = hwMap.get(Servo.class, "plane");

        motors = new DcMotor[]{
                frontLeft,
                frontRight,
                backLeft,
                backRight,
                slideLeft,
                slideRight,
                intake
        };
        driveMotors = new DcMotor[]{
                frontLeft,
                frontRight,
                backLeft,
                backRight
        };`


        //armB = hwMap.get(DcMotor.class, "armB");

   /*     BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;j
       // parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    */
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        slideLeft.setDirection(DcMotor.Direction.REVERSE);
        slideRight.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);

        // Set all motors to zero power, make them reset, set their zero pwr behavior to brake

        for (DcMotor motor : motors) {
            motor.setPower(0);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }
}

