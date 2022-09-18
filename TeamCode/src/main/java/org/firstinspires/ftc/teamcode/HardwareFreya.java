package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class HardwareFreya {

    /* Public OpMode members. */
    public DcMotor leftDrive   = null;
    public DcMotor  rightDrive  = null;
    public DcMotor  lift     = null;
    public Servo grab    = null;
    public DcMotor spin   = null;
    BNO055IMU imu;

    //public static final double MID_SERVO       =  0.5 ;
    //public static final double ARM_UP_POWER    =  0.45 ;
    //public static final double ARM_DOWN_POWER  = -0.45 ;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwareFreya(){

    }

    //Motor Constants
    static final int ENCODER_CPR_60 = 1680;
    static final int ENCODER_CPR_40 = 1120;
    static final int REV_ENCODER = 288;
    static final int HIGH_LIFT = ENCODER_CPR_60/12;
    static final double LOW_LIFT = 0;
    static final int MOVE_ONE = HIGH_LIFT/3;
    static final double WHEEL_DIAMETER_INCHES = 3.5;
    static final double WHEEL_CPI = REV_ENCODER/(WHEEL_DIAMETER_INCHES*Math.PI);

    //Servo Constants
    static final double GRAB_OPEN = 0.8;
    static final double GRAB_CLOSE = 0.9;

    //IMU Constants
    Orientation ANGLES;
    Orientation LAST_ANGLE = new Orientation();
    double GLOBAL_ANGLE;
    public ElapsedTime runtime = new ElapsedTime();
    public ElapsedTime maxtime = new ElapsedTime();


    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        leftDrive  = hwMap.get(DcMotor.class, "left_drive");
        rightDrive = hwMap.get(DcMotor.class, "right_drive");
        lift    = hwMap.get(DcMotor.class, "lift");
        spin    = hwMap.get(DcMotor.class, "spinner");

        //Define and Initialize Servos
        grab  = hwMap.get(Servo.class, "grab");

        //Set Directions for Motors
        leftDrive.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        rightDrive.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        lift.setDirection(DcMotorSimple.Direction.FORWARD);

        // Set all motors to zero power
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        lift.setPower(0.0);
        spin.setPower(0);

        // Set encoder functionality for motors
        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        spin.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Set Zero Power Behavior
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setPower(0.0);


    }

    public void tele_init (HardwareMap teleHwMap) {
        init(teleHwMap);

        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void auto_init (HardwareMap autoHwMap) {
        init(autoHwMap);

        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


    }

    public void initIMU() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }

    public void resetAngle() {
        LAST_ANGLE = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        GLOBAL_ANGLE = 0;
    }

    public double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        ANGLES = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = ANGLES.firstAngle - LAST_ANGLE.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        GLOBAL_ANGLE += deltaAngle;

        LAST_ANGLE = ANGLES;

        return GLOBAL_ANGLE;
    }
}
