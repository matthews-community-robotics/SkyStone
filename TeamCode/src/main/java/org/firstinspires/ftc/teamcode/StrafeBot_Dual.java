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

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 * The code is structured as a LinearOpMode
 *
 * This particular OpMode executes a POV Game style Teleop for a PushBot
 * In this mode the left stick moves the robot FWD and back, the Right stick turns left and right.
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="StrafeBot OPMode", group="StrafeBot")

public class StrafeBot_Dual extends LinearOpMode {


    BNO055IMU imu;
    /* Declare OpMode members. */
    HardwareStrafeBot robot = new HardwareStrafeBot();   // Use a Pushbot's hardware     // sets rate to move servo


    public double controlMod(double value){
        return Math.abs(Math.pow(value,2)) * value;
    }

    @Override
    public void runOpMode() {
        //double leftFront = 0;
        //double leftRear = 0;
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);


        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        // run until the end of the match (driver presses STOP)

        while (opModeIsActive()) {

            //Original code taken from: https://ftcforum.usfirst.org/forum/ftc-technology/android-studio/6361-mecanum-wheels-drive-code-example
            double r = Math.hypot(controlMod(gamepad1.right_stick_x), controlMod(gamepad1.left_stick_y));
            double robotAngle = Math.atan2(controlMod(gamepad1.left_stick_y), controlMod(gamepad1.right_stick_x)) - Math.PI / 4;
            double rightX = controlMod(gamepad1.left_stick_x);
            final double v1 = r * Math.cos(robotAngle) + rightX; //LFRONT
            final double v2 = r * Math.sin(robotAngle) - rightX; //RFRONT
            final double v3 = r * Math.sin(robotAngle) + rightX; //LBACK
            final double v4 = r * Math.cos(robotAngle) - rightX; //RBACK

            /*if(gamepad1.left_stick_y > 0.2 || gamepad1.left_stick_y < -0.2){
                robot.lfront.setPower(-(gamepad1.left_stick_y-0.2));
                robot.rfront.setPower((gamepad1.left_stick_y-0.2));
                robot.lback.setPower((gamepad1.left_stick_y-0.2));
                robot.rback.setPower(-(gamepad1.left_stick_y-0.2));
            }else {
            robot.lfront.setPower(v1);
            robot.rfront.setPower(-v2);
            robot.lback.setPower(v3);
            robot.rback.setPower(-v4);*/
            //}

            robot.lfront.setPower(v1);
            robot.rfront.setPower(v2);
            robot.lback.setPower(v3);
            robot.rback.setPower(v4);
            /*if(gamepad1.dpad_up){
                robot.lfront.setPower(0.5);
            }
            if(gamepad1.dpad_right){
                robot.rfront.setPower(0.5);
            }
            if(gamepad1.dpad_down){
                robot.rback.setPower(0.5);
            }
            if(gamepad1.dpad_left){
                robot.lback.setPower(0.5);
            }*/

            telemetry.addData("Robot Angle: ", robotAngle);
            telemetry.update();
            // Pace this loop so jaw action is reasonable speed.
            sleep(50);
        }
    }
}
