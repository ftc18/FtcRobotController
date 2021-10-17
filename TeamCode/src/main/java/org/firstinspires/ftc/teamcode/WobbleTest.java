package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name="Wobble", group="Linear Opmode")
public class WobbleTest extends LinearOpMode {

    public Servo wobble = null;
    public DcMotor wobbleArm = null;

    @Override
    public void runOpMode() {

        wobble = hardwareMap.get(Servo.class, "wobble");
        wobbleArm = hardwareMap.get(DcMotor.class,"wobbleArm");

        waitForStart();
        while (opModeIsActive()) {

            if(gamepad1.a) {
                wobble.setPosition(0.25);
            }
            else if(gamepad1.y) {
                wobble.setPosition(1.00);
            }
        }
    }
}
