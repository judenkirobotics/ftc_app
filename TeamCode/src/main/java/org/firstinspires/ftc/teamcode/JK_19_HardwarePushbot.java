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
 */package org.firstinspires.ftc.teamcode;
import android.graphics.drawable.GradientDrawable;
import android.os.SystemClock;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left drive              "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class JK_19_HardwarePushbot
{
    /* Public OpMode members. */
    public DcMotor     leftDrive             = null;
    public DcMotor     rightDrive            = null;
    public DcMotor     mineralMotor          = null;
    public Servo       ColorSensingServo     = null;
    public ColorSensor MineralColorSensor    = null;
    public CRServo     PaddleServo           = null;
    public Servo       FlipServo             = null;
    public DcMotor     LiftMotor             = null;

    //IMU test
    public BNO055IMU   imu                   = null;
    Orientation        lastAngles            = new Orientation();
    double             globalAngle           = 0.3;
    double             power                 = 0.3;
   // public Servo       HookServo           =null;
    //public Servo       arm                 = null;
   // public DcMotor     leftRear            = null;
    //public DcMotor     rightRear           = null;
   // public DcMotor     rampMotor           = null;
    //public TouchSensor extensionTouch      = null;  // Ramp Deployment
    //public DcMotor     extensionMotor      = null;
    //public DcMotor     loaderMotor         = null;
    //public GyroSensor  gyro                = null;
    //Testing relic
    //public Servo       relicPivotServo     = null;
    //public Servo       relicClawServo      = null;
    //public DcMotor     relicExtendMotorCmd = null;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public JK_19_HardwarePushbot(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap, boolean useGyro) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        leftDrive  = hwMap.get(DcMotor.class, "Left");
        rightDrive = hwMap.get(DcMotor.class, "Right");
        mineralMotor  = hwMap.get(DcMotor.class, "mineralMotor");
        FlipServo  = hwMap.get(Servo.class,"FlipServo");
        LiftMotor = hwMap.get(DcMotor.class, "LiftMotor");


        //leftArm    = hwMap.get(DcMotor.class, "left_arm");
        leftDrive.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        rightDrive.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        mineralMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        // Set all motors to zero power
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        mineralMotor.setPower(0);


        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mineralMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        ColorSensingServo = hwMap.get(Servo.class, "ColorSensingServo");
        MineralColorSensor = hwMap.get(ColorSensor.class, "MineralColorSensor");
        PaddleServo = hwMap.get(CRServo.class, "PaddleServo");
        MineralColorSensor.enableLed(false);
        ColorSensingServo.setDirection(Servo.Direction.FORWARD);
        ColorSensingServo.setPosition(0.0);
        PaddleServo.setDirection(DcMotorSimple.Direction.FORWARD);
        PaddleServo.setPower(0);
        FlipServo.setDirection(Servo.Direction.FORWARD);
        FlipServo.setPosition(1.0);
        LiftMotor.setPower(0);
        LiftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        LiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LiftMotor.setTargetPosition(0);

        //IMU Test

        if (useGyro) {
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.mode                  = BNO055IMU.SensorMode.IMU;
            parameters.angleUnit             = BNO055IMU.AngleUnit.DEGREES;
            parameters.accelUnit             = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.loggingEnabled        = false;
            imu = hwMap.get(BNO055IMU.class, "imu");  //Must be device 0 on i2c 0
            imu.initialize(parameters);
            int i = 0;
            while (!imu.isGyroCalibrated() && i<60) {
                i++;
                SystemClock.sleep(100);
            }
        }





        //Define and Initialize Sensors
     //   gyro = hwMap.get(GyroSensor.class, "gyro");
     //   extensionTouch = hwMap.get(TouchSensor.class, "ext_touch");
    }
 }



















