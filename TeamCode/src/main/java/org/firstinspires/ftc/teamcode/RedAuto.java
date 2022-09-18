/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.sun.tools.javac.util.Abort;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

/**
 * This file illustrates the concept of driving up to a line and then stopping.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code shows using two different light sensors:
 *   The Primary sensor shown in this code is a legacy NXT Light sensor (called "sensor_light")
 *   Alternative "commented out" code uses a MR Optical Distance Sensor (called "sensor_ods")
 *   instead of the LEGO sensor.  Chose to use one sensor or the other.
 *
 *   Setting the correct WHITE_THRESHOLD value is key to stopping correctly.
 *   This should be set half way between the light and dark values.
 *   These values can be read on the screen once the OpMode has been INIT, but before it is STARTED.
 *   Move the senso on asnd off the white line and not the min and max readings.
 *   Edit this code to make WHITE_THRESHOLD half way between the min and max.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

/**@Autonomous(name="Red autonomous", group="Linear auto")
public class RedAuto extends LinearOpMode {

    HardwareFreya robot = new HardwareFreya();

    @Override
    public void runOpMode() {

        robot.auto_init(hardwareMap);
        robot.initIMU();

        /* Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */

       /** // If there are encoders connected, switch to RUN_USING_ENCODER mode for greater accuracy
        // robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // get a reference to our Light Sensor object.
        //  lightSensor = hardwareMap.opticalDistanceSensor.get("sensor_ods");  // Alternative MR ODS sensor.

        // turn on LED of light sensor.

        // Send telemetry message to signify robot waiting;
        telemetry.addData("IMU?", robot.getAngle());    //
        telemetry.update();

/*        while (robot.getAngle() <= 90) {
            telemetry.addData("IMU?", robot.getAngle());    //
            telemetry.update();
        }*/

        /** Wait for the game to start (driver presses PLAY)
        waitForStart();

        moveThatRobot(30,34.5,0.5,1000,5000);
        rotateRobot(90,0.3);
        //Abort this loop is started or stopped.
        while (!(isStarted() || isStopRequested())) {

             //Display the light level while we are waiting to start
            telemetry.update();
            idle();
        }

        // Start the robot moving forward, and then begin looking for a white line.


/*        // run until the white line is seen OR the driver presses STOP;
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 4)) {

            // Display the light level while we are looking for the line
            leftDrive.setPower(1.0);
            rightDrive.setPower(1.0);
            telemetry.update();
        }*/

       /** // Stop all motors

    }

    public void moveThatRobot (double rightDistance, double leftDistance, double speed, long timeout, int max) {

        robot.runtime.reset();
        int newLeftTarget;
        int newRightTarget;

        if (opModeIsActive()) {
            robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



            newLeftTarget = robot.leftDrive.getCurrentPosition() + (int) (leftDistance * robot.WHEEL_CPI);
            newRightTarget = robot.rightDrive.getCurrentPosition() + (int) (rightDistance * robot.WHEEL_CPI);

            robot.leftDrive.setTargetPosition(newLeftTarget);
            robot.rightDrive.setTargetPosition(newRightTarget);


            robot.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            //Turn on motors
            robot.leftDrive.setPower(speed);
            robot.rightDrive.setPower(speed);

            robot.maxtime.reset();
            while (opModeIsActive() && robot.leftDrive.isBusy() && robot.rightDrive.isBusy()
                    && (robot.maxtime.milliseconds() < max)) {

            }

            //Stopping motors
            robot.leftDrive.setPower(0);
            robot.rightDrive.setPower(0);

            //Turn off RUN_TO_POSITION
            robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(timeout);
        }

    }

    void rotateRobot(int degrees, double power) {
        double leftPower;
        double rightPower;

        // restart imu movement tracking.
        robot.resetAngle();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        if (degrees < 0) {   // turn right.
            leftPower = -power;
            rightPower = power;
        } else if (degrees > 0) {   // turn left.
            leftPower = power;
            rightPower = -power;
        } else return;

        // set power to rotate.
        robot.leftDrive.setPower(leftPower);
        robot.rightDrive.setPower(rightPower);

        // rotate until turn is completed.
        if (degrees < 0) {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && robot.getAngle() == 0) {
            }

            while (opModeIsActive() && robot.getAngle() > degrees) {
            }
        } else    // left turn.
            while (opModeIsActive() && robot.getAngle() < degrees) {
            }

        // turn the motors off.
       // robot.rightDrive.setPower(0);
        //robot.leftDrive.setPower(0);

        // wait for rotation to stop.
        //sleep(1000);

        // reset angle tracking on new heading.
        //robot.resetAngle();
**/


