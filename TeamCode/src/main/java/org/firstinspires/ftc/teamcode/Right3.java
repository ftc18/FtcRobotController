package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Right3", group="Robot")
//@Disabled
public class Right3 extends LinearOpMode {

    private final ElapsedTime runtime = new ElapsedTime();
    EosHardware robot = new EosHardware(this);

    @Override
    public void runOpMode() {robot.init();
        telemetry.addData("Status", "Ready to run");
        telemetry.update();
        waitForStart();
        robot.setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.init();
        robot.setServoPosition(robot.SERVOCLOSED);
        robot.straightByEncoder(1,36, 10);
        robot.turnToHeading(1, 45);
        robot.straightByEncoder(-1,6,5);
        robot.turnToHeading(-1, 45);
        robot.slideTimed(1,5);
        robot.setServoPosition(robot.SERVOOPENED);

    }
}
