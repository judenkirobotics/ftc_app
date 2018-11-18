/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;
import org.firstinspires.ftc.teamcode.KernelPanic;

//import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

/**

 */

//@Autonomous(name="Time Slice Op Mode 2", group="Pushbot")
    //@TeleOp(name = "Time Slice Op Mode 2", group = "HardwarePushbot")
//@Disabled
public class timeSliceOpMode extends LinearOpMode {

    /* Declare OpMode members. */
    private KernelPanic robot = new KernelPanic();   // Use a Pushbot's hardware

    private static final double COUNTS_PER_MOTOR_REV = 1440;    // eg: TETRIX Motor Encoder
    private static final double DRIVE_GEAR_REDUCTION = 2.0;     // This is < 1.0 if geared UP
    private static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    private static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    private static final double DRIVE_SPEED = 0.6;
    private static final double TURN_SPEED = 0.5;


    /* Public OpMode members. */
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor liftMotor = null;

    // Define class members
    public Servo leftClamp = null;
    public Servo rightClamp = null;

    static final double INCREMENT = 0.01;     // amount to slew servo each CYCLE_MS cycle
    static final int CYCLE_MS = 50;     // period of each cycle
    static final double MAX_POS = 1.0;     // Maximum rotational position
    static final double MIN_POS = 0.0;     // Minimum rotational position

    double clampOffset = 0;                       // Servo mid position
    final double CLAMP_SPEED = 0.02;                   // sets rate to move servo
    final long SENSORPERIOD = 50;
    final long ENCODERPERIOD = 50;
    final long SERVOPERIOD = 50;
    final long NAVPERIOD = 50;
    final long MOTORPERIOD = 50;
    final long CONTROLLERPERIOD = 50;
    final long TELEMETRYPERIOD = 1000;


    double position = (MAX_POS - MIN_POS) / 2; // Start at halfway position
    boolean rampUp;

    {
        rampUp = true;
    }

    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();


        ElapsedTime runtime = new ElapsedTime();
        DcMotor leftMotor = null;
        DcMotor rightMotor = null;

        //A Timing System By Katherine Jeffrey,and Alexis
        // long currentThreadTimeMillis (0);
        //

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        /***************************************************************************
         *            Everything below here  \\ press START           *
         ***************************************************************************/
        //@Override

        long CurrentTime = System.currentTimeMillis();

        long LastSensor = CurrentTime;
        long LastEncoderRead = CurrentTime + 5;
        long LastServo = CurrentTime + 10;
        long LastNav = CurrentTime + 15;
        long LastMotor = CurrentTime + 20;
        long LastController = CurrentTime + 7;
        long LastTelemetry = CurrentTime + 17;

        // variables for controller inputs.
        float g1_leftX;
        float g1_LeftY;
        float g1_RightX;
        float g1_RightY;
        boolean g1_A;
        boolean g1_B;

        // variables to support clamp and lift
        long clampStart = 0;
        long liftStart = 0;

        //double legTime = CurrentTime;
        //double lastTelemetry = CurrentTime;
        double timeLeft = 0;
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            CurrentTime = System.currentTimeMillis();

            //Loop For Timing System
            /* ***************************************************
             *                SENSORS
             *        INPUTS: Raw Sensor Values
             *       OUTPUTS: parameters containing sensor values*
             ****************************************************/
            if (CurrentTime - LastSensor > SENSORPERIOD) {
                LastSensor = CurrentTime;

            }


            /* ***************************************************
             *                ENCODERS                          *
             ****************************************************/
            if (CurrentTime - LastEncoderRead > ENCODERPERIOD) {
                LastEncoderRead = CurrentTime;
                // We want to READ the Encoders here
                //    ONLY set the motors in motion in ONE place.



            }
            /* **************************************************
             *                Controller INPUT                  *
             *  INPUTS: raw controller values                   *
             *  OUTPUTS:
             *         g1_LeftX    g1_RightX
             *         g1_LeftY    g1_RightY
             *         g1_a (gamepad A)
             *         g1_b (gamepad B)
             ****************************************************/
            if (CurrentTime - LastController > CONTROLLERPERIOD) {
                LastController = CurrentTime;
                g1_leftX = gamepad1.left_stick_x;
                g1_LeftY = gamepad1.left_stick_y;
                g1_RightX = gamepad1.right_stick_x;
                g1_RightY = gamepad1.right_stick_y;

                g1_A = gamepad1.a;
                g1_B = gamepad1.b;
/*  ***********************************************************************
     ^^^^^^^^^^^^^ ALL OF THE STUFF ABOVE HERE IS READING INPUTS ^^^^^^^^^^^^^^^
 ***************************************************************************/




/* vvvvvvvvvvvvvvvvvv  THIS SECTION IS MAPPING INPUTS TO OUTPUTS vvvvvvvvvvvvvvvvv*/

            /* **************************************************
             *                NAV
             *      Inputs:  Gamepad positions
             *               Sensor Values (as needed)
             *      Outputs: Servo and Motor position commands
             *                         motor
             ****************************************************/
                if (CurrentTime - LastNav > NAVPERIOD) {
                    LastNav = CurrentTime;
                    // mapping inputs to servo command
                    if (g1_A) {
                        //leftClamp;
                        clampOffset += CLAMP_SPEED;
                    } else if (gamepad1.b) {
                        clampOffset -= CLAMP_SPEED;
                    }
                    // mapping inputs to motor commands

                }
/*  ^^^^^^^^^^^^^^^^  THIS SECTION IS MAPPING INPUTS TO OUTPUTS ^^^^^^^^^^^^^^^*/




/*  ***********************************************************************
                 ALL OF THE STUFF BELOW HERE IS WRITING OUTPUTS
 * VVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVV*/

            /* **************************************************
             *                SERVO
             *                Inputs: leftClamp position command
             *                        rightClamp position command *
             ****************************************************/
                if (CurrentTime - LastServo > SERVOPERIOD) {
                    LastServo = CurrentTime;
                    // Use gamepad left & right Bumpers to open and close the thing
                    if (gamepad1.a) {
                        clampOffset += CLAMP_SPEED;
                        liftMotor.setPower(0.5);
                    } else if (gamepad1.b) {
                        clampOffset -= CLAMP_SPEED;
                        liftMotor.setPower(-0.5);
                    }

                    // Move both servos to new position.  Assume servos are mirror image of each other.
                    clampOffset = Range.clip(clampOffset, -0.5, 0.5);
                    leftClamp.setPosition(robot.MID_SERVO + clampOffset);
                    rightClamp.setPosition(robot.MID_SERVO - clampOffset);
                }


            /* ***************************************************
             *                MOTOR OUTPUT
             *       Inputs:  Motor power commands
             *       Outputs: Physical interface to the motors
             ****************************************************/
                if (CurrentTime - LastMotor > MOTORPERIOD) {
                    LastMotor = CurrentTime;


                    if (gamepad1.left_stick_y != 0) {
                        leftDrive.setPower(gamepad1.left_stick_y);
                        rightDrive.setPower(gamepad1.left_stick_y);
                    } else if (gamepad1.left_stick_x != 0) {


                    }
                }


            /* ***************************************************
             *                TELEMETRY
             *       Inputs:  telemetry structure
             *       Outputs: command telemetry output to phone
             ****************************************************/

                if (CurrentTime - LastTelemetry > TELEMETRYPERIOD) {
                    LastTelemetry = CurrentTime;
                    telemetry.update();
                }
            }

            telemetry.addData("Path", "Complete");
            telemetry.update();
        }
    }
}
