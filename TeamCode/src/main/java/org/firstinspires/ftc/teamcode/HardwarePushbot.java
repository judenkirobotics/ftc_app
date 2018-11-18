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

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 v *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class HardwarePushbot
{
    /* Public OpMode members. */
    public DcMotor  leftDrive   = null;
    public DcMotor  rightDrive  = null;
    public DcMotor  leftRear   = null;
    public DcMotor  rightRear  = null;
    public DcMotor  mineralfront = null;
    public DcMotor  encodelift   = null;
    //public DcMotor  pulleyDrive     = null;
    //public Servo    leftClamp    = null;
    //public Servo    rightClamp   = null;
    //public SensorDigitalTouch touchSensor = null;
    //public SensorMROpticalDistance leftDistance = null;
    //public SensorMROpticalDistance rightDistance = null;

    public static final double ARM_UP_POWER    =  0.45 ;
    public static final double ARM_DOWN_POWER  = -0.45 ;
    public static final double MID_SERVO = 0.45;


/*
    public static final double LEFTCLAMPED = 0.92;
    public static final double LEFTMOSTLYCLAMPED = 0.75;
    public static final double LEFTUNCLAMPED = .25;
    public static final double LEFTTIGHTCLAMPED = .99;
    public static final double RIGHTCLAMPED = 0.92;
    public static final double RIGHTUNCLAMPED = .25;
    public static final double RIGHTMOSTLYCLAMPED = 0.75;
    public static final double RIGHTTIGHTCLAMPED = .99;
    public static final double SERVO_TWEAK = .005;
*/


    public static final double LEFTCLAMPED = 0.25;
    public static final double LEFTMOSTLYCLAMPED = 0.37;
    public static final double LEFTUNCLAMPED = .80;
    public static final double LEFTTIGHTCLAMPED = .20;
    public static final double RIGHTCLAMPED = 0.25;
    public static final double RIGHTUNCLAMPED = .80;
    public static final double RIGHTMOSTLYCLAMPED = 0.37;
    public static final double RIGHTTIGHTCLAMPED = .20;
    public static final double SERVO_TWEAK = .005;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwarePushbot(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        leftDrive  = hwMap.get(DcMotor.class, "left_drive");
        rightDrive = hwMap.get(DcMotor.class, "right_drive");
        leftRear  = hwMap.get(DcMotor.class, "left_rear");
        rightRear = hwMap.get(DcMotor.class, "right_rear");
        mineralfront = hwMap.get (DcMotor.class, "mineral_front");
        encodelift = hwMap.get (DcMotor.class, "encode_lift");


        //leftDistance = hwMap.get(SensorMROpticalDistance.class,"left_distance");
        //rightDistance = hwMap.get(SensorMROpticalDistance.class,"right_distance");
        //pulleyDrive    = hwMap.get(DcMotor.class, "pulley");
        leftDrive.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        rightDrive.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        leftRear.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        rightRear.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        mineralfront.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        encodelift.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors

        // Set all motors to zero power
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        mineralfront.setPower(0);
        encodelift.setPower(0);
        //pulleyDrive.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mineralfront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        encodelift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      //  pulleyDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Define and initialize ALL installed servos.
        /*leftClamp  = hwMap.get(Servo.class, "left_arm");
        rightClamp = hwMap.get(Servo.class, "right_arm");
        leftClamp.setDirection(Servo.Direction.FORWARD);
        rightClamp.setDirection(Servo.Direction.REVERSE);
        leftClamp.setPosition(LEFTUNCLAMPED);   //May need to set to full open to stay in 18" cube
        rightClamp.setPosition(RIGHTUNCLAMPED); */

    }
 }

