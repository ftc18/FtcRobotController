package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Tele-Op", group="Linear Opmode")
public class Tele extends LinearOpMode {

    // Declare OpMode members.
    //private ElapsedTime runtime = new ElapsedTime();
    HardwareFreya robot = new HardwareFreya();

    @Override
    public void runOpMode() {
        robot.tele_init(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        //runtime.reset();
        telemetry.addLine("Lift:" + robot.lift.getCurrentPosition());
        telemetry.update();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup variables for each motor to save power level for telemetry


            // Tank Mode uses one stick to control each wheel.
            // - This requires no math, but it is hard to drive forward slowly and keep straight.
            double leftPower  = gamepad1.left_stick_y ;
            double rightPower = gamepad1.right_stick_y ;
            double spinPower = gamepad2.right_stick_y;


            // Send calculated power to wheels
            robot.leftDrive.setPower(leftPower);
            robot.rightDrive.setPower(rightPower);
            robot.spin.setPower(spinPower);

            if (gamepad2.y) {
                telemetry.addLine("Current:" + robot.lift.getCurrentPosition());
                telemetry.update();
                int up_pos = robot.lift.getCurrentPosition() + robot.MOVE_ONE;
                telemetry.addLine("New Position:" + up_pos);
                telemetry.update();
                if (up_pos <= robot.HIGH_LIFT) {
                    robot.lift.setTargetPosition(up_pos);
                    robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.lift.setPower(0.8);
                }
                robot.lift.setPower(0.0);
            }
// run until the end of the match (driver presses STOP)
            double tgtPower = 0;
            while (opModeIsActive()) {
                if(gamepad2.a) {
                    // move to 0 degrees.
                    robot.grab.setPosition(0);
                } else  {
                    robot.grab.setPosition(0.5);
                }
//                telemetry.addData("Servo Position", servoTest.getPosition());
//                telemetry.addData("Target Power", tgtPower);
//                telemetry.addData("Motor Power", motorTest.getPower());
//                telemetry.addData("Status", "Running");
//                telemetry.update();

            }
            if (gamepad2.a) {
                int down_pos = robot.lift.getCurrentPosition() - robot.MOVE_ONE;
                robot.lift.setTargetPosition(down_pos);
                robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.lift.setPower(0.2);
            }

            if (gamepad2.y)
            // Show the elapsed game time and wheel power.
            //telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.update();
        }
    }
}

