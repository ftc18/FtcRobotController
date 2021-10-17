package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Double Tele-Op", group="Linear Opmode")
public class DoubleTele extends LinearOpMode {

    Hardware robot = new Hardware();

    @Override
    public void runOpMode() {

        robot.teleInit(hardwareMap);

        waitForStart();

        while(opModeIsActive()) {
            double leftPower = -gamepad1.left_stick_y;
            double rightPower = -gamepad1.right_stick_y;
            double inPower = -gamepad2.right_stick_y;
            double conPower = -gamepad2.left_stick_y;
            float outPower = gamepad2.right_trigger;

            robot.frontRight.setPower(rightPower);
            robot.frontLeft.setPower(leftPower);
            robot.backRight.setPower(rightPower);
            robot.backLeft.setPower(leftPower);

            while(gamepad1.a) {
                telemetry.addLine("A Pressed");
                telemetry.update();
                robot.wobble.setPower(0.5);
            }
            while(gamepad1.y) {
                telemetry.addLine("Y Pressed");
                telemetry.update();
                robot.wobble.setPower(-0.65);
            }

            if(gamepad2.dpad_up) {
                robot.wobbleArm.setTargetPosition(robot.WOBBLE_UP);
                robot.wobbleArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.wobbleArm.setPower(0.5);
            }
            else if (gamepad2.dpad_down) {
                robot.wobbleArm.setTargetPosition(robot.WOBBLE_DOWN);
                robot.wobbleArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.wobbleArm.setPower(0.4);
            }

            robot.intake.setPower(inPower);
            robot.conveyor.setPower(conPower);

            robot.rOut.setPower(outPower*5);
            robot.lOut.setPower(outPower*5);


        }
    }
}
