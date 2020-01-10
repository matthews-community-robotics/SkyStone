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
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */



public class HardwareStrafeAutoBot
{



    //Vars-----------------------------
    private ElapsedTime runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 288;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 1.22*3.5;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    BNO055IMU imu;
    Orientation angles;
    //---------------------------------

    public void moveEncoder(double inches, double power, LinearOpMode Caller){
        lfront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lfront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        inches = -inches;
        inches = inches*55.555555555;
        lfront.setPower(-power);
        lback.setPower(-power);
        rfront.setPower(-power);
        rback.setPower(-power);
        while (Caller.opModeIsActive() && lfront.getCurrentPosition() > inches){
            Caller.telemetry.addData("Current: ", lfront.getCurrentPosition());
        }
        lfront.setPower(0);
        lback.setPower(0);
        rfront.setPower(0);
        rback.setPower(0);
    }

    public void moveBackwardsEncoder(double inches, double power, LinearOpMode Caller){
        lfront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lfront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        inches = inches;
        inches = inches*55.555555555;
        lfront.setPower(-power);
        lback.setPower(-power);
        rfront.setPower(-power);
        rback.setPower(-power);
        while (Caller.opModeIsActive() && lfront.getCurrentPosition() < inches){
            Caller.telemetry.addData("Current: ", lfront.getCurrentPosition());
        }
        lfront.setPower(0);
        lback.setPower(0);
        rfront.setPower(0);
        rback.setPower(0);
    }

    public void strafeLeft(double inches, double power, LinearOpMode Caller){
        lfront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lfront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        inches = 1.2 * inches;
        inches = inches*55.555555555;
        lfront.setPower(power);
        lback.setPower(power);
        rfront.setPower(-power);
        rback.setPower(-power);
        while (Caller.opModeIsActive() && lfront.getCurrentPosition() < inches){
            Caller.telemetry.addData("Current: ", lfront.getCurrentPosition());
        }
        lfront.setPower(0);
        lback.setPower(0);
        rfront.setPower(0);
        rback.setPower(0);
    }

    public void strafeRight(double inches, double power, LinearOpMode Caller){
        lfront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lfront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        inches = 1.2 * inches;
        inches = -inches;
        inches = inches*55.555555555;
        lfront.setPower(-power);
        lback.setPower(-power);
        rfront.setPower(power);
        rback.setPower(power);
        while (Caller.opModeIsActive() && lfront.getCurrentPosition() > inches){
            Caller.telemetry.addData("Current: ", lfront.getCurrentPosition());
        }
        lfront.setPower(0);
        lback.setPower(0);
        rfront.setPower(0);
        rback.setPower(0);
    }

    //Check IMU Angles
    private double checkOrientation() {
        // read the orientation of the robot
        angles = this.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        this.imu.getPosition();
        // and save the heading
        double curHeading = angles.firstAngle;
        return -curHeading;
    }
    //Gyro/IMU Turn
    public void turnIMU(boolean isHeading, double degrees, double power, LinearOpMode Caller){
        if(degrees < 0){
            degrees+=10;
        }else{
            degrees-=10;
        }
        double targetDegree = 0;
        if(isHeading){
            targetDegree = degrees;
        } else{
            targetDegree = checkOrientation() + degrees;
        }

        if(targetDegree < checkOrientation()) {
            lfront.setPower(power);
            lback.setPower(-power);
            rfront.setPower(-power);
            rback.setPower(power);
            while (Caller.opModeIsActive() && checkOrientation() > targetDegree){
                Caller.telemetry.addData("Current Degrees: ", checkOrientation());
                Caller.telemetry.addData("Turn Target Degrees: ", targetDegree);
                Caller.telemetry.update();
            }
        }else{
            lfront.setPower(- power);
            lback.setPower(power);
            rfront.setPower(power);
            rback.setPower(-power);
            while (Caller.opModeIsActive() && checkOrientation() < targetDegree){
                Caller.telemetry.addData("Current Degrees: ", checkOrientation());
                Caller.telemetry.addData("Turn Target Degrees: ", targetDegree);
                Caller.telemetry.update();
            }
        }
        lfront.setPower(0);
        lback.setPower(0);
        rfront.setPower(0);
        rback.setPower(0);
    }
    //Move using Motor Encoder



    /* Public OpMode members. */
    public DcMotor  lfront = null;
    public DcMotor  lback  = null;
    public DcMotor  rfront   = null;
    public DcMotor  rback  = null;
    public CRServo arm = null;
    public Servo claw = null;
    //public Servo    rightClaw   = null;

    //public static final double MID_SERVO       =  0.5 ;
    //public static final double ARM_UP_POWER    =  0.45 ;
    //public static final double ARM_DOWN_POWER  = -0.45 ;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwareStrafeAutoBot(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        lfront  = hwMap.get(DcMotor.class, "lfront");
        lback = hwMap.get(DcMotor.class, "lback");
        rfront  = hwMap.get(DcMotor.class, "rfront");
        rback = hwMap.get(DcMotor.class, "rback");
        arm = hwMap.get(CRServo.class, "arm");

        claw = hwMap.get(Servo.class, "claw");
        //leftArm    = hwMap.get(DcMotor.class, "left_arm");

        lfront.setDirection(DcMotor.Direction.REVERSE);
        rback.setDirection(DcMotor.Direction.REVERSE);

        // Set all motors to zero power
        lfront.setPower(0);
        lback.setPower(0);
        rfront.setPower(0);
        rback.setPower(0);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // Define and initialize ALL installed servos.
        /*leftClaw  = hwMap.get(Servo.class, "left_hand");
        rightClaw = hwMap.get(Servo.class, "right_hand");
        leftClaw.setPosition(MID_SERVO);
        rightClaw.setPosition(MID_SERVO);*/
    }
}

