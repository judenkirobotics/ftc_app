package org.firstinspires.ftc.teamcode;

/* *
 * Created by ftcrobotics on 11/19/17.
 *
 * Concept by Howard
 *
 * First Coding by Jeffrey and Alexis
 *
 *
 *
 */

//0, 50, 0, 50, 0

/*
        import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
        import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
        import com.qualcomm.robotcore.hardware.DcMotor;
        import com.qualcomm.robotcore.hardware.Servo;
        import com.qualcomm.robotcore.util.ElapsedTime;
        import com.qualcomm.robotcore.util.Range;

        import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;
*/
//import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;
/*Made by Tarun and John*/

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ClassUtil;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//@Autonomous(name="Pushbot: Auto Drive By Encoder", group="Pushbot")

@Autonomous(name="JK Autonomous Crater Side", group="Pushbot")
@SuppressWarnings("WeakerAccess")
//@TeleOp(name = "Time Slice Op Mode", group = "HardwarePushbot")
//@Disabled
public class JK_AutonomousCrater extends LinearOpMode {

    JK_19_HardwarePushbot robot   = new JK_19_HardwarePushbot();   // Use a Pushbot's hardware
    final long SENSORPERIOD  = 30;
    final long ENCODERPERIOD = 20;
    final long SERVOPERIOD  =  20;
    final long NAVPERIOD    =  20;
    final long MOTORPERIOD  =  20;
    final long CONTROLLERPERIOD = 20;
    final long TELEMETRYPERIOD = 500;
    final double MAX_S_POS = 0.85;
    final double SWING_TRANSIT = 0.55;

    boolean gold_detected = false;
    boolean gold_third_pos = false;
    boolean swing_complete = false;

    float stageTime = 0;
    final int SWG = 0;
    final int FWD = 1;
    final int KIK = 2;
    final int PADDLE = 3;
    final int ARM_HOME = 4;
    final int ARM_MAX = 5;
    final int PAUSE = 6;
    final int WAIT = 7; // MUST be highest numbered state!!
    final float MAX_RED_GREEN = (float)1.75;
    final float MIN_RED_GREEN = (float)1.2;
    final float MAX_BLUE_GREEN = (float)0.65;
    final float MIN_BLUE_GREEN = (float)0.0;
    final double SWING_HOME = 0.1;
    final long KIK_TRANSIT_TIME = 220;
    final long WALL_WARNING = 1000;
    final long SWING_TRANSIT_TIME = KIK_TRANSIT_TIME + 100;

    int CurrentAutoState = 0;
    //Initial code
    //int[] stage =       {FWD, SWG,   KIK, KIK, SWG, FWD,  FWD, FWD, FWD,  WAIT};
    //double[] l_power =  {-0.5,  0,     0,  0,    0, -0.5, -0.3,  0.5, 0.4,    0};
    //double[] r_power =  {-0.5,  0,     0,  0,    0,  0.5, -0.3, -0.5, 0.4,    0};
    //long[] paddle    =  { 0,    0,     1, -1,    0,    0,   -1,    0,   0,    0};
    //double[] stageLim = {440, 4500, 1200, 1000, 250, 700, 1450,  550, 1000, 20000};

    int[] stage =       {FWD, SWG,   KIK,  ARM_HOME,  PADDLE, FWD,   FWD,  ARM_MAX,      KIK, ARM_HOME,  FWD,  WAIT,    FWD,  FWD,  FWD,   WAIT};
    double[] l_power =  {-0.5,  0,     0,         0,  0,     -0.5,  -0.5,        0,        0,        0,  -0.5,  -0.5,  -0.5,    1,   -1,  0};
    double[] r_power =  {-0.5,  0,     0,         0,  0,      0.7,  -0.5,        0,        0,        0,   0.7,  -0.5,   0.7,    1,   -1,  0};
    long[] paddle    =  { 0,    0,     1,         0,  0,       0,     0,        0,        1,        0,     0,     0,     0,    0,    0,  0};
    double[] stageLim = {460, 4500, 1200,      1000, 200,     600,   340,      500,     1000,      500,   100,  1600,   1600, 1200,  2800, 20000};

    //double[]paddleTime = { 0,    0,   0,    0,  1000,    0};
    int gold_position = 1;
    int red;
    int green;
    int blue;
    double sPos = 0.0;
    double pPower = 0.0;
    double leftMotorCmd = 0.0;
    double rightMotorCmd = 0.0;

    //@Override
    public boolean detectGold(int r, int g, int b){
        boolean gold = false; //default value
        if (g > 0){
            if (((float)(r)/(float)(g) < MAX_RED_GREEN) &&
                ((float)(r)/(float)(g) > MIN_RED_GREEN) &&
                ((float)(b)/(float)(g) < MAX_BLUE_GREEN)){
                gold = true; // changes to true only if the if statement is satisfied
            }
        }
        return (gold);
    }
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        long CurrentTime = System.currentTimeMillis();
        long LastSensor = CurrentTime;
        long LastServo = CurrentTime;
        long LastNav = CurrentTime;
        long LastMotor = CurrentTime;
        long LastTelemetry = CurrentTime;

        sPos = robot.ColorSensingServo.getPosition();  // Set initial value

        telemetry.addData("Status   ", "Initialized");
        telemetry.addData("SPos ", sPos);
        telemetry.update();

        ElapsedTime runtime = new ElapsedTime();
        //A Timing System By Katherine Jeffrey,and Alexis
        // Wait for the game to start (driver presses PLAY)

        waitForStart();
        runtime.reset();
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
                red = robot.MineralColorSensor.red();
                green = robot.MineralColorSensor.green();
                blue = robot.MineralColorSensor.blue();
            }

        /*  ***********************************************************************
         ^^^^^^^^^^^^^ ALL OF THE STUFF ABOVE HERE IS READING INPUTS ^^^^^^^^^^^^^^^
        /* vvvvvvvvvvvv  THIS SECTION IS MAPPING INPUTS TO OUTPUTS vvvvvvvvvvvvvvvvv*/

            /* **************************************************
             *                NAV
             *      Inputs:  Gamepad positions
             *               Sensor Values (as needed)
             *      Outputs: Servo and Motor position commands
             *                         motor
             ****************************************************/
            if (CurrentTime - LastNav > NAVPERIOD) {
                LastNav = CurrentTime;
                boolean stage_complete = false;
                stageTime += NAVPERIOD;
                telemetry.addData("Current index: ", CurrentAutoState);
                telemetry.addData("Current State: ", stage[CurrentAutoState]);
                telemetry.addData("sPos        ", sPos);
                telemetry.addData("RED: ", red);
                telemetry.addData("GREEN: ", green);
                telemetry.addData("BLUE: ", blue);
                telemetry.addData("gold_detected ", gold_detected);
                telemetry.addData("swing_complete ", swing_complete);
                telemetry.addData("gold_position ", gold_position);

                switch (stage[CurrentAutoState]) {
                    case FWD:
                        leftMotorCmd = l_power[CurrentAutoState];
                        rightMotorCmd = r_power[CurrentAutoState];
                        break;
                    // Swing arm looking for gold block, if found knock it off
                    case SWG:
                        gold_detected = gold_detected || detectGold(red, green, blue);

                        if (gold_detected == true) {
                            stage_complete = true;
                        }
                        else if (sPos >= MAX_S_POS) {
                            stage_complete = true;
                            gold_third_pos = true;
                        }
                        else  {
                            sPos += 0.004;
                        }
                        break;
                    case KIK:
                        if (gold_detected)  {
                            pPower = paddle[CurrentAutoState];
                            gold_detected = false;
                        }
                        if (gold_third_pos && (CurrentAutoState > 3)) {
                            pPower = paddle[CurrentAutoState];
                            gold_third_pos = false;
                        }
                        break;
                    case PADDLE:
                        pPower = paddle[CurrentAutoState];
                        break;

                    case ARM_HOME:
                        sPos = SWING_HOME;
                        break;
                    case ARM_MAX:
                        sPos = MAX_S_POS;
                        break;
                    case PAUSE:
                        telemetry.addData("Pausing...", red);
                        break;
                    case WAIT:
                        pPower = 0;
                        leftMotorCmd = 0.0;
                        rightMotorCmd = 0.0;
                        telemetry.addData("Waiting... ", red);
                        break;
                    default:
                        break;
                }

                /*  ORIGINAL CODE
                switch (stage[CurrentAutoState]) {
                    case FWD:
                        if ((!gold_detected) && (swing_complete) && (gold_position ==1)){

                            if (stageTime > SWING_TRANSIT_TIME) {
                                sPos = SWING_TRANSIT;
                            }
                            if (stageTime > KIK_TRANSIT_TIME) {
                                pPower = paddle[CurrentAutoState];
                            }

                            leftMotorCmd = l_power[CurrentAutoState];
                            rightMotorCmd = r_power[CurrentAutoState];
                        }
                        if (stageTime > WALL_WARNING) {
                            sPos = SWING_HOME;
                            pPower = paddle[CurrentAutoState];
                        }
                        break;
                    // Swing arm looking for gold block, if found knock it off
                    case SWG:
                        boolean prev_gold_detected = gold_detected;
                        gold_detected = gold_detected || detectGold(red, green, blue);

                        if ((sPos >= MAX_S_POS) || (gold_detected && !prev_gold_detected)) {
                            stage_complete = true;
                            swing_complete = true;
                            if (gold_detected) gold_position = (sPos < .6)? 2: 3;
                        }
                        else if (!prev_gold_detected) {
                            sPos = sPos + 0.003;
                        }
                        else {
                            sPos = SWING_HOME;
                        }
                        break;
                    case KIK:
                        if ((!gold_detected) && (gold_position > 1)){
                            stage_complete = true;
                        }
                        else {
                            pPower = paddle[CurrentAutoState];
                        }
                        break;

                    case WAIT:
                        pPower = 0;
                        leftMotorCmd = 0.0;
                        rightMotorCmd = 0.0;
                        telemetry.addData("Waiting... ", red);
                        break;
                    default:
                        break;
                }

                    ORIGINAL CODE*/


                if ((stageTime >= stageLim[CurrentAutoState]) ||
                        (stage_complete)){
                    stageTime = 0;
                    leftMotorCmd = 0;
                    rightMotorCmd = 0;
                    pPower = 0;
                    if (stage[CurrentAutoState] < WAIT) {
                        CurrentAutoState ++;
                    }
                }
            }
            // END NAVIGATION

        /*   ^^^^^^^^^^^^^^^^  THIS SECTION IS MAPPING INPUTS TO OUTPUTS   ^^^^^^^^^^^^^^^*/
        /* ********************************************************************************/
        /*  ***********************************************************************
         * VVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVV
         *           ALL OF THE STUFF BELOW HERE IS WRITING OUTPUTS
         * VVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVV*/

            /* ***************************************************
             *                MOTOR OUTPUT
             *       Inputs:  Motor power commands
             *       Outputs: Physical interface to the motors
             ****************************************************/
            if (CurrentTime - LastMotor > MOTORPERIOD) {
                LastMotor = CurrentTime;

                robot.leftDrive.setPower(leftMotorCmd);
                robot.rightDrive.setPower(rightMotorCmd);

            }

            /* ***************************************************
             *                SERVO OUTPUT
             *       Inputs:  Motor power commands
             *       Outputs: Physical interface to the servos
             ****************************************************/
            if (CurrentTime - LastServo > SERVOPERIOD) {
                LastServo = CurrentTime;
                robot.ColorSensingServo.setPosition(sPos);
                robot.PaddleServo.setPower(pPower);
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
        } // end of while opmode is active

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

}