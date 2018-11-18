package org.firstinspires.ftc.teamcode;

/* *
 * Created by ftcrobotics on 11/19/17.
 *
 * Concept by Howard
 *
 * First Coding by Jeffrey and Alexis
 *
 * Editing for the 2018-2019 season by Tarun and Jacob
 *
 */

import android.os.SystemClock;

import com.qualcomm.hardware.ams.AMSColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.NextLock;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

//@Autonomous(name="Time Slice Op Mode 1", group="Pushbot")
@SuppressWarnings("WeakerAccess")
@TeleOp(name = "Time Slice Mecanum", group = "HardwarePushbot")
//@Disabled
public class KP_Driver_Mecanum extends LinearOpMode {
    //Encoder enc;
    //enc = new Encoder(0,1,false,Encoder.EncodingType.k4X);
    /* Declare OpMode members. */
    HardwarePushbot robot   = new HardwarePushbot();   // Use a Pushbot's hardware
    static final double CLAMP_MOTION_TIME = 250;

    final long SENSORPERIOD = 50;
    final long ENCODERPERIOD = 50;
    final long SERVOPERIOD = 50;
    final long NAVPERIOD = 50;
    final long MOTORPERIOD = 50;
    final long CONTROLLERPERIOD = 50;
    final long TELEMETRYPERIOD = 1000;


    final int RISER_DISTANCE = 500;
    final int BLUE_LED_CHANNEL = 0;
    final int RED_LED_CHANNEL = 1;


    int rightMotorPos;
    int leftMotorPos;
    int mineralfrontMotorPos;
    int encodeflipMotorPos;
    int prevRiserErr = 0;

    //double position = (MAX_POS - MIN_POS) / 2; // Start at halfway position

    //@Override

    /*public botMotors makeAsquare(double dur) {
        botMotors sq = new botMotors();
        sq.leftFront = (float)-1;
        sq.rightFront = (float)1;
        if (dur > 3000)
        {
            sq.leftFront = (float)-0.5;
            sq.rightFront = (float)-0.5;
        }
        else if (dur > 2000) {
            sq.leftFront = 1;
            sq.rightFront = -1;
        }
        else if (dur > 1000) {
            sq.leftFront = (float)0.5;
            sq.rightFront = (float)0.5;
        }


    }
    */

    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        boolean inputPin = false;             // Input State
        boolean outputPin;            // Output State
        //DeviceInterfaceModule dim;                  // Device Object
        //DigitalChannel        digIn;                // Device Object
        //DigitalChannel        digOut;               // Device Object

        // get a reference to a Modern Robotics DIM, and IO channels.
        // dim = hardwareMap.get(DeviceInterfaceModule.class, "dim");   //  Use generic form of device mapping
        //digIn  = hardwareMap.get(DigitalChannel.class, "digin");     //  Use generic form of device mapping
        //digOut = hardwareMap.get(DigitalChannel.class, "digout");    //  Use generic form of device mapping

        //digIn.setMode(DigitalChannel.Mode.INPUT);          // Set the direction of each channel
        //digOut.setMode(DigitalChannel.Mode.OUTPUT);

        //

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();
        //botMotors mDrive = new botMotors();
        long CurrentTime = System.currentTimeMillis();

        long LastSensor = CurrentTime;
        long LastEncoderRead = CurrentTime + 5;
        long LastServo = CurrentTime + 10;
        long LastNav = CurrentTime + 15;
        long LastMotor = CurrentTime + 20;
        long LastController = CurrentTime + 7;
        long LastTelemetry = CurrentTime + 17;

        long liftDuration = 0;
        long liftOffDuration = 0;

        // variables for controller inputs.
        float g1_LeftX = 0;
        float g1_LeftY = 0;
        float g1_RightX = 0;
        float g1_RightY = 0;
        boolean g1_A = false;
        boolean g1_B = false;
        boolean g1_X = false;
        boolean g1_Y = false;
        boolean g1_DD = false;
        boolean g1_DU = false;
        boolean g1_DL = false;
        boolean g1_DR = false;
        boolean g1_LB = false;
        boolean g1_RB = false;
        float g1_LT = 0;
        float g1_RT = 0;

        float g2_leftX = 0;
        float g2_LeftY = 0;
        float g2_RightX = 0;
        float g2_RightY = 0;
        boolean g2_A = false;
        boolean g2_B = false;
        boolean g2_X = false;
        boolean g2_Y = false;
        boolean g2_DD = false;
        boolean g2_DU = false;
        boolean g2_DL = false;
        boolean g2_DR = false;
        boolean g2_LB = false;
        boolean g2_RB = false;
        float g2_LT = 0;
        float g2_RT = 0;


        int g2_A_Counts = 0;
        int g2_DU_Counts = 0;


        float leftDriveCmd = 0;
        float rightDriveCmd = 0;
        float leftRearCmd = 0;
        float rightRearCmd = 0;
        float leftDriveCrab = 0;
        float rightDriveCrab = 0;
        float leftRearCrab = 0;
        float rightRearCrab = 0;
        float mineralfrontCmd = 0;
        float encodeflipCmd = 0;

        float turtleScaler = 1;  //Initially full power
        float turtleSpeed = 4;  // Divider

        float g1_LT_Threshold = (float) 0.4;
        float g1_X_Threshold = (float) 0.4;
        float g1_Y_Threshold = (float) 0.4;
        float g2_X_Threshold = (float) 0.4;
        float g2_Y_Threshold = (float) 0.4;
        float g1_Crab_Threshold = (float) 0.3;

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        ElapsedTime runtime = new ElapsedTime();


        //A Timing System By Katherine Jeffrey,and Alexis
        // long currentThreadTimeMillis (0);
        //
        //   int riserZero = robot.pulleyDrive.getCurrentPosition();


        waitForStart();
        runtime.reset();
  /* ***********************************************************************************************
   *****************************                CODE          ************************************
   ************************************************************************************************/


        /* ************************************************************
         *            Everything below here  \\ press START           *
         **************************************************************/

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
                // We want to READ the Encoders here  Not currently using data, invalid
                //    ONLY set the motors in motion in ONE place.
              //  mineralfrontMotorPos = robot.mineralfront.getCurrentPosition();
              //  rightMotorPos = robot.rightDrive.getCurrentPosition();
              //  leftMotorPos = robot.leftDrive.getCurrentPosition();
                encodeflipMotorPos = robot.encodeflip.getCurrentPosition();
                //  riserotorPos = robot.pulleyDrive.getCurrentPosition();

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

                g1_LeftX = gamepad1.left_stick_x;
                g1_LeftY = gamepad1.left_stick_y;
                g1_RightX = gamepad1.right_stick_x;
                g1_RightY = gamepad1.right_stick_y;
                g1_A = gamepad1.a;
                g1_B = gamepad1.b;
                g1_X = gamepad1.x;
                g1_Y = gamepad1.y;
                g1_DD = gamepad1.dpad_down;
                g1_DU = gamepad1.dpad_up;
                g1_DL = gamepad1.dpad_left;
                g1_DR = gamepad1.dpad_right;
                g1_LB = gamepad1.left_bumper;
                g1_RB = gamepad1.right_bumper;
                g1_LT = gamepad1.left_trigger;
                g1_RT = gamepad1.right_trigger;

                g2_leftX = gamepad2.left_stick_x;
                g2_LeftY = gamepad2.left_stick_y;
                g2_RightX = gamepad2.right_stick_x;
                g2_RightY = gamepad2.right_stick_y;
                g2_A = gamepad2.a;
                g2_B = gamepad2.b;
                g2_X = gamepad2.x;
                g2_Y = gamepad2.y;
                g2_DD = gamepad2.dpad_down;
                g2_DU = gamepad2.dpad_up;
                g2_DL = gamepad2.dpad_left;
                g2_DR = gamepad2.dpad_right;
                g2_LB = gamepad2.left_bumper;
                g2_RB = gamepad2.right_bumper;
                g2_LT = gamepad2.left_trigger;
                g2_RT = gamepad2.right_trigger;


                // do a little debounce on gamepad2.a so we don't drop the block accidentally
                // 6 counts at 30 milliseconds will delay things by 180 ms, and that allows
                // a flaky controller or a jittery operator. Splitting out the sensor "reads"
                // from the rest of the logic let's us do this here, rather than muddling up
                // the main logic of a more abstract, more complicated piece of code.
                g2_A_Counts = Range.clip((gamepad2.a) ? g2_A_Counts + 1 : g2_A_Counts - 2, 0, 12);
                g2_A = (g2_A_Counts >= 6);

                g2_DU_Counts = Range.clip((gamepad2.dpad_up) ? g2_DU_Counts + 1 : g2_DU_Counts - 2, 0, 12);
                g2_DU = (g2_DU_Counts >= 6);



        /*  ***********************************************************************
         ^^^^^^^^^^^^^ ALL OF THE STUFF ABOVE HERE IS READING INPUTS ^^^^^^^^^^^^^^^
         ***************************************************************************/



        /* ********************************************************************************/
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

                    // init drive min and max to default values.  We'll reset them to other numbers
                    // if conditions demand it.
                    float driveMax = 1;
                    float driveMin = -1;

                    leftDriveCmd = 0;
                    rightDriveCmd = 0;
                    leftRearCmd = 0;
                    rightRearCmd = 0;
                    leftDriveCrab = 0;
                    rightDriveCrab = 0;
                    leftRearCrab = 0;
                    rightRearCrab = 0;

                    // ********************************
                    //Motor to take in minerals. mineralfront
                    if (g2_LT > 0.15) {
                        mineralfrontCmd = (float) (-0.75);
                    } else if (g2_RT > 0.15) {
                        mineralfrontCmd = (float) (0.75);
                    } else {
                        mineralfrontCmd = (float) (0);
                        mineralfrontCmd = Range.clip(mineralfrontCmd, (float) driveMin, (float) driveMax);
                    }
                    // End of Mineral Motor Commands
                    // ********************************

                    //Encoded lift Commands
                    //*********************************
                    //robot.encodeflip.setTargetPosition(0);
                    /*
                    //This code is what ran an infinite loop, but so does the current code.

                       robot.encodeflip.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                       robot.encodeflip.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                       robot.encodeflip.setTargetPosition(+1680);
                       robot.encodeflip.setPower(.5);

                       while ((robot.encodeflip.isBusy()) &&opModeIsActive() )  {

                       }
                       robot.encodeflip.setPower(0);
                    */
                    if (g2_RB == true) {
                        robot.encodeflip.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        robot.encodeflip.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        robot.encodeflip.setDirection(DcMotorSimple.Direction.FORWARD);
                        robot.encodeflip.setTargetPosition(1000);
                        robot.encodeflip.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        robot.encodeflip.setPower(.5);
                        while (robot.encodeflip.isBusy() && opModeIsActive()) {
                            telemetry.addData("counts  ", robot.encodeflip2.getCurrentPosition());
                            telemetry.update();

                        }
                    }




                    if (g2_LB == true)  {
                            robot.encodeflip2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                            robot.encodeflip2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                            robot.encodeflip2.setDirection(DcMotorSimple.Direction.FORWARD);
                            robot.encodeflip2.setTargetPosition(1000);
                            robot.encodeflip2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            robot.encodeflip2.setPower(.5);
                            while (robot.encodeflip2.isBusy() && opModeIsActive()) {
                                telemetry.addData("counts#2  ", robot.encodeflip2.getCurrentPosition());
                                telemetry.update();


                        }
                    }


                    else {
                            robot.encodeflip2.setPower(0);
                        }


                    encodeflipCmd = Range.clip(encodeflipCmd,(float)driveMin, (float)driveMax);
                   //End of Encoded Lift Commands



                    if (Math.abs(g1_LeftX) < g1_X_Threshold) {
                        //Move forwards or backwards
                        leftDriveCmd = Range.clip(g1_LeftY * g1_LeftY * g1_LeftY, driveMin, driveMax);
                        rightDriveCmd = leftDriveCmd;
                        leftRearCmd = leftDriveCmd;
                        rightRearCmd = leftDriveCmd;
                    }

                    if (Math.abs(g1_LeftX) > g1_X_Threshold) {
                        //Pivot
                        leftDriveCmd = Range.clip(g1_LeftX * g1_LeftX * g1_LeftX, driveMin, driveMax);
                        rightDriveCmd = -1 * Range.clip(g1_LeftX * g1_LeftX * g1_LeftX, driveMin, driveMax);
                        leftRearCmd = leftDriveCmd;
                        rightRearCmd = rightDriveCmd;


                        if ((Math.abs(g1_RightY) > g1_Crab_Threshold) || (Math.abs(g1_RightX) > g1_Crab_Threshold)) {

                            if (Math.abs(g1_RightX) < g1_Crab_Threshold) {  //Forward, backward
                                leftDriveCrab = Range.clip(g1_RightY * g1_RightY * g1_RightY, driveMin, driveMax);
                                rightDriveCrab = leftDriveCrab;
                                leftRearCrab = leftDriveCrab;
                                rightRearCrab = leftDriveCrab;
                            } else {
                                leftDriveCrab = -1 * Range.clip(g1_RightX * g1_RightX * g1_RightX, driveMin, driveMax);
                                rightDriveCrab = Range.clip(g1_RightX * g1_RightX * g1_RightX, driveMin, driveMax);
                                leftRearCrab = rightDriveCrab;
                                rightRearCrab = leftDriveCrab;
                            }

                        }


                    }                    // END NAVIGATION


        /*   ^^^^^^^^^^^^^^^^  THIS SECTION IS MAPPING INPUTS TO OUTPUTS   ^^^^^^^^^^^^^^^*/
        /* ********************************************************************************/



        /*  ***********************************************************************
         * VVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVV
         *           ALL OF THE STUFF BELOW HERE IS WRITING OUTPUTS
         * VVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVV*/


            /* **************************************************
             *                SERVO OUTPUT
             *                Inputs: leftClamp position command
             *                        rightClamp position command *
             *                Outputs: Physical write to servo interface.
             ****************************************************/
                    if (CurrentTime - LastServo > SERVOPERIOD) {
                        LastServo = CurrentTime;

                        // Move both servos to new position.
                        //   robot.leftClamp.setPosition(leftClamp_Cmd);
                        //   robot.rightClamp.setPosition(rightClamp_Cmd);
                    }


            /* ***************************************************
             *                MOTOR OUTPUT
             *       Inputs:  Motor power commands
             *       Outputs: Physical interface to the motors
             ****************************************************/
                    if (CurrentTime - LastMotor > MOTORPERIOD) {
                        LastMotor = CurrentTime;
                        // Yes, we'll set the power each time, even if it's zero.
                        // this way we don't accidentally leave it somewhere.  Just simpler this way.
                    /*  Left Drive Motor Power  */
                        //robot.leftDrive.setPower(leftDriveCmd);
                       // robot.leftDrive.setPower(-1 * (leftDriveCmd + leftDriveCrab) / turtleScaler);
                       // robot.rightDrive.setPower(-1 * (rightDriveCmd + rightDriveCrab) / turtleScaler);
                      //  robot.leftRear.setPower(-1 * (leftRearCmd + leftRearCrab) / turtleScaler);
                      //  robot.rightRear.setPower(-1 * (rightRearCmd + rightRearCrab) / turtleScaler);

                        // mineral motor power set
                        //robot.mineralfront.setPower(mineralfrontCmd);

                    /* Lifter Motor Power   */
                        //   robot.pulleyDrive.setPower(riserCmd);
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

                //telemetry.addData("Left Motor Power     ", -1 * (leftDriveCmd + leftDriveCrab) / turtleScaler);
                //telemetry.addData("Right Motor Power    ", -1 * (rightDriveCmd + rightDriveCrab) / turtleScaler);
                //telemetry.addData("Left Rear Power     ", -1 * (leftRearCmd + leftRearCrab) / turtleScaler);
                //telemetry.addData("Right Rear Power    ", -1 * -1 * (rightRearCmd + rightRearCrab) / turtleScaler);
                //telemetry.addData("mineralfront Power ", -1 * -1);
                telemetry.addData("motor position ", robot.encodeflip.getTargetPosition());
                telemetry.update();
            }


            //SAFE EXIT OF RUN OPMODE, stop motors, leave servos????
            // robot.pulleyDrive.setPower(0);
            //robot.leftDrive.setPower(0);
            //robot.rightDrive.setPower(0);
            //robot.leftRear.setPower(0);
            //robot.rightRear.setPower(0);
           // robot.mineralfront.setPower(0);
        }
    }
}
