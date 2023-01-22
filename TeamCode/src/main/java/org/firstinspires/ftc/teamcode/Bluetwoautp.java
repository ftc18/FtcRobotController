package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="bluetwo", group="Robot")
//@Disabled
public class Bluetwoautp extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    EosHardware robot = new EosHardware(this);

    @Override
    public void runOpMode() {
        robot.init();
        telemetry.addData("Status", "Ready to run");
        waitForStart();
        robot.setServoPosition(robot.SERVOCLOSED);
        robot.stop();
        robot.straightTimed(-.5,3.4);
        runtime.reset();
        robot.strafeTimed(-.5, .5, 1.01);
        robot.slideTimed(1, 5.09);
        robot.straightTimed(-.5, .01);
        robot.setServoPosition(robot.SERVOOPENED);
    }
}
