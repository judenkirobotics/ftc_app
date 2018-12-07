package org.firstinspires.ftc.teamcode;

/* *
 * Created by ftcrobotics on 11/19/17.
 *
 * Concept by Howard
 *
 * First Coding by Jeffrey and Alexis
 *
 * Modified for the 2019 season by Tarun
 *
 */


        import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
        import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
        import com.qualcomm.robotcore.hardware.DcMotor;
        import com.qualcomm.robotcore.hardware.DcMotorSimple;
        import com.qualcomm.robotcore.hardware.Servo;
        import com.qualcomm.robotcore.util.ElapsedTime;
        import com.qualcomm.robotcore.util.Range;

        import org.firstinspires.ftc.teamcode.HardwarePushbot;

//import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

//import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

import java.util.concurrent.atomic.AtomicIntegerArray;
//@Autonomous(name="Pushbot: Auto Drive By Encoder", group="Pushbot")

@Autonomous(name="Autonomous DepotSide", group="Pushbot")
@SuppressWarnings("WeakerAccess")
//@TeleOp(name = "Time Slice Op Mode", group = "HardwarePushbot")
//@Disabled
public class KPAutonomousDepot extends LinearOpMode {
    /* Declare OpMode members. */
    HardwarePushbot robot   = new HardwarePushbot();   // Use a Pushbot's hardware


    final int  AUTO_STATES = 4;
    final long SENSORPERIOD = 50;
    final long ENCODERPERIOD = 50;
    final long SERVOPERIOD = 50;
    final long NAVPERIOD = 50;
    final long MOTORPERIOD = 50;
    final long CONTROLLERPERIOD = 50;
    final long TELEMETRYPERIOD = 1000;

    final double PROPGAIN = 0.6;
    final double INTGAIN  = 0.3;
    final double DERGAIN  = 0.1;
    final long PIDMAXDUR  = 3;

    int currState = 0;




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
        final float driveMax = 1;
        final float driveMin = -1;
        long CurrentTime = System.currentTimeMillis();

        long LastSensor = CurrentTime;
        long LastEncoderRead = CurrentTime + 5;
        long LastServo = CurrentTime + 10;
        long LastNav = CurrentTime + 15;
        long LastMotor = CurrentTime + 20;
        long LastController = CurrentTime + 7;
        long LastTelemetry = CurrentTime + 17;


        double leftDriveCmd = 0;
        double rightDriveCmd = 0;
        double leftRearCmd = 0;
        double rightRearCmd = 0;
        double pulleyCmd = 0;
        double flipperCmd = 0;
        double mineralCmd = 0;

        //
        //       0    1     2   3     4    5    6   7
        final int FLIPPER = 0;
        final int PULLEY = 1;
        final int FWD = 2;
        final int CRAB = 3;
        final int COLOR = 4;
        final int TURN = 5;
        final int PAUSE = 6;
        final int WAIT = 100;  //Make larger than # of stages to run

        float stageTime = 0;
        int CurrentAutoState = 0;
        //int[] stage =       {FWD, CRAB, FWD, PULLEY, FLIPPER, TURN, FWD, FWD, WAIT};
        int[] stage =       {FWD, CRAB, TURN, CRAB, FWD, PULLEY, PAUSE, FLIPPER, FLIPPER, FWD,  WAIT};
        double[] stageLim = {700, 2000, 1000, 1500, 950,   1200,   500,    1000,    1500, 2000,  30000};
        double[] mtrPower = {0.7, 1.0,  0.7,  1.0,  1.0,   -1.0,   0.0,    -1.0,     1.0, -1.0,  0.0};


        telemetry.addData("Status", "Initialized");
        telemetry.update();

        ElapsedTime runtime = new ElapsedTime();

        // Wait for the game to start (driver presses PLAY)
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
                // no sensors at this time.  If we add some, change this comment.
            }


            /* ***************************************************
             *                ENCODERS                          *
             ****************************************************/
            if (CurrentTime - LastEncoderRead > ENCODERPERIOD) {
                LastEncoderRead = CurrentTime;


            }
            /* **************************************************
             *                Controller INPUT                  *
             *  INPUTS: raw controller values                   *
             *  OUTPUTS:                                        *
             *         NO CONTROLLER INPUT FOR AUTONOMOUS       *
             ********* THIS COMMENT IS JUST A REFERENCE**********/
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
                boolean stage_complete = false;
                stageTime += NAVPERIOD;
                telemetry.addData("Current index: ", CurrentAutoState);
                telemetry.addData("Current state: ", stage[CurrentAutoState]);

                switch (stage[CurrentAutoState]) {
                    case FLIPPER: // Depley Flipper
                        flipperCmd = mtrPower[CurrentAutoState];
                        break;
                    case PULLEY:  // Move Pulley
                        pulleyCmd = mtrPower[CurrentAutoState];
                        break;
                    case FWD:// Drive Forward
                        leftDriveCmd =  mtrPower[CurrentAutoState];
                        rightDriveCmd = mtrPower[CurrentAutoState];
                        leftRearCmd =   mtrPower[CurrentAutoState];
                        rightRearCmd =  mtrPower[CurrentAutoState];
                        break;
                    case CRAB:   // Drive Crab
                        leftDriveCmd =   mtrPower[CurrentAutoState];
                        rightDriveCmd = -1 * mtrPower[CurrentAutoState];
                        leftRearCmd =   -1 * mtrPower[CurrentAutoState];
                        rightRearCmd =   mtrPower[CurrentAutoState];
                        break;
                    case COLOR: // Detect Gold, also drives and flips
                        break;
                    case TURN: // Drive Turn
                        leftDriveCmd =  -mtrPower[CurrentAutoState];
                        rightDriveCmd = mtrPower[CurrentAutoState];
                        leftRearCmd =   -mtrPower[CurrentAutoState];
                        rightRearCmd =  mtrPower[CurrentAutoState];
                        break;
                    case PAUSE: // Just chill
                        telemetry.addData("Pausing...  ", stageTime);
                        break;
                    case WAIT: // We are done!!!
                        telemetry.addData("Waiting...  ", stageTime);
                        break;
                    default:
                        break;

                }

                if ((stageTime >= stageLim[CurrentAutoState]) || (stage_complete)) {
                    stageTime = 0;
                    leftDriveCmd = 0;
                    rightDriveCmd = 0;
                    leftRearCmd = 0;
                    rightRearCmd = 0;
                    pulleyCmd = 0;
                    flipperCmd = 0;
                    mineralCmd = 0;
                    if (stage[CurrentAutoState] < WAIT) {
                        CurrentAutoState++;
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


            /* **************************************************
             *                SERVO OUTPUT
             *                Inputs: leftClamp position command
             *                        rightClamp position command *
             *                Outputs: Physical write to servo interface.
             ****************************************************/
            if (CurrentTime - LastServo > SERVOPERIOD) {
                LastServo = CurrentTime;

                // Move both servos to new position.
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

                robot.leftDrive.setPower(leftDriveCmd);
                robot.leftRear.setPower(leftRearCmd);
                robot.rightDrive.setPower(rightDriveCmd);
                robot.rightRear.setPower(rightRearCmd);
                robot.encodeFlip.setPower(flipperCmd);
                robot.encodelift.setPower(pulleyCmd);
                robot.mineralfront.setPower(mineralCmd);
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

        // Disable all motors for safe exit from autonomous
        robot.leftDrive.setPower(0.0);
        robot.leftRear.setPower(0.0);
        robot.rightDrive.setPower(0.0);
        robot.rightRear.setPower(0.0);
        robot.encodeFlip.setPower(0.0);
        robot.encodelift.setPower(0.0);
        robot.mineralfront.setPower(0.0);


        }

    }
