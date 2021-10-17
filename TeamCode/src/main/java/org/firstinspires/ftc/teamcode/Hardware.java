package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Hardware {

    //Public OpMode Members
    public DcMotor frontLeft = null;
    public DcMotor frontRight = null;
    public DcMotor backLeft = null;
    public DcMotor backRight = null;
    public CRServo wobble = null;
    public DcMotor wobbleArm = null;
    public CRServo intake = null;
    public DcMotor conveyor = null;
    public DcMotor rOut = null;
    public DcMotor lOut = null;

    //Motor Constants
    static final int ENCODER_CPR_60 = 1680;

    //Wobble Constants
    static final double WOBBLE_OPEN = 1.0;
    static final double WOBBLE_CLOSE = 0.25;
    static final int WOBBLE_UP = 0;
    static final int WOBBLE_DOWN = (ENCODER_CPR_60/2)-100;


    /*Local OpMode Members*/
    HardwareMap hwMap = null;

    /*Constructor*/
    public Hardware() {
    }

    /*Initialize Standard Hardware*/
    public void init(HardwareMap aHwMap) {
        hwMap = aHwMap;

        //Define and Initialize Motors and Servos
        frontLeft = hwMap.get(DcMotor.class,"frontLeft");
        frontRight = hwMap.get(DcMotor.class,"frontRight");
        backLeft = hwMap.get(DcMotor.class,"backLeft");
        backRight = hwMap.get(DcMotor.class,"backRight");

        wobble = hwMap.get(CRServo.class,"wobble");
        wobbleArm = hwMap.get(DcMotor.class,"wobbleArm");

        intake = hwMap.get(CRServo.class,"intake");
        conveyor = hwMap.get(DcMotor.class,"conveyor");

        rOut = hwMap.get(DcMotor.class,"rOut");
        lOut = hwMap.get(DcMotor.class,"lOut");


    }

    public void teleInit(HardwareMap otherHardwareMap) {
        init(otherHardwareMap);

        //Set Drive Directions
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        wobble.setDirection(DcMotorSimple.Direction.FORWARD);
        wobbleArm.setDirection(DcMotor.Direction.FORWARD);
        wobbleArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wobbleArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wobbleArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        conveyor.setDirection(DcMotor.Direction.FORWARD);

        rOut.setDirection(DcMotor.Direction.FORWARD);
        lOut.setDirection(DcMotor.Direction.REVERSE);

    }
}
