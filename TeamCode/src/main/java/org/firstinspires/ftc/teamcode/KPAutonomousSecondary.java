package org.firstinspires.ftc.teamcode;

/* *
 * Created by ftcrobotics on 11/19/17.
 *
 * Concept by Howard
 *
 * First Coding by Jeffrey and Alexis
 *
 *
 *Edited for the 2019 season by Tarun and Jacob
 */



 import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;
//@Autonomous(name="Pushbot: Auto Drive By Encoder", group="Pushbot")

//@Autonomous(name="Autonomous BLUE", group="Pushbot")
@SuppressWarnings("WeakerAccess")
//@TeleOp(name = "Time Slice Op Mode", group = "HardwarePushbot")
//@Disabled
public class KPAutonomousSecondary extends LinearOpMode {
    /* Declare OpMode members. */
    HardwarePushbot robot   = new HardwarePushbot();   // Use a Pushbot's hardware

    /* Public OpMode members. */
  /*  static final double LEFTCLAMPED = 45;
    static final double LEFTUNCLAMPED = -5;
    static final double RIGHTCLAMPED = 5;
    static final double RIGHTUNCLAMPED = -45;

    static final double CLAMP_MOTION_TIME = 250;
*/
    final int  AUTO_STATES = 4;
    final long SENSORPERIOD = 50;
    final long ENCODERPERIOD = 50;
    final long SERVOPERIOD = 50;
    final long NAVPERIOD = 50;
    final long MOTORPERIOD = 50;
    final long TELEMETRYPERIOD = 1000;


    final double PROPGAIN = 0.6;
    final double INTGAIN  = 0.3;
    final double DERGAIN  = 0.1;
    final long PIDMAXDUR  = 3;

    final int RISER_DISTANCE = 500;
    int currState = 0;
    int rightMotorPos;
    int lefMotorPos;
    int riserMotorPos;
    int prevRiserErr = 0;

    //double position = (MAX_POS - MIN_POS) / 2; // Start at halfway position

    public double simplePID (double err, double duration, double prevErr)
    {
        double pidmin = -.7;
        double pidmax = 0.7;
        double propTerm = Range.clip(PROPGAIN * err, pidmin, pidmax);
        double intTerm = Range.clip(duration/PIDMAXDUR,pidmin,pidmax)*INTGAIN;
        double derTerm = Range.clip((err - prevErr),pidmin,pidmax) * DERGAIN;
        return (Range.clip(propTerm + intTerm + derTerm, pidmin,pidmax));
    }
    public boolean detectItem() {
        // presuming there will be a detect item here ... populate this code when we know that
        return true;
    }
    public boolean moveLever() {
        // presuming we will move a lever somehow.  Populate this method when that is known.
        return true;
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
        final float driveMax = 1;
        final float driveMin = -1;
        final float riserMax = 1;
        final float riserMin = -1;
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
        //float g1_leftX;
        float g1_LeftY;
        //float g1_RightX;
        float g1_RightY;
        int g1_A_Counts = 0;

      //  double leftClamp_Cmd = LEFTUNCLAMPED;
        //double rightClamp_Cmd = RIGHTUNCLAMPED;
        double stageTimer=0;

        float startTime = 0;
        int startHeading = 0;
        double startPos = 0;
        float leftDriveCmd = 0;
        float rightDriveCmd = 0;
        float leftRearCmd = 0;
        float rightRearCmd = 0;
        float riserCmd=0;
        // 0: ball, 1: clamp, 2: lift, 3: fwd, 4: crab,
        // 5: fwd,  6: unclamp, 7: reverse, 8: wait.
        //                    0    1     2   3     4    5    6   7
        //int CLAMP = 1;
        //int FWD = 2;
        //int CRAB = 3;
        //int[] thisStage =   {BL, CLM, LFT, FWD, CRB, FWD, CLM, FWD}
        double[] timeLimit = {100,  500, 600, 750, 750, 600, 500, 300, 30000};
        //int[] TurnArray =    {0,   45,   0,   2,   0,   0,   0,   0};
        //int[] TurnPower =    {0,   40,   0, -30,   0,  0,   0,   0};
        //float[] StraightPwr= {0,    0,  30,   0,   0,  0,   0,   0};
        //int[] StraightDist=  {0,    0,  50,   0,   0,  0,   0,   0};
        //int[] crabArray =    {0,   45,   0,   2,   0,  0,   0,   0};

        int    clampMaxTime = 500;
        int    driveMaxTime = 2000;   //Crude two seconds, eventually use encoders
        int    driveBackMaxTime = 200;
        int    driveForwardLittleTime = 1000;
        int    driveBackLittleTime = 250;

        boolean g1_A;
        //boolean g1_B;
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        ElapsedTime runtime = new ElapsedTime();
        //A Timing System By Katherine Jeffrey,and Alexis
        // long currentThreadTimeMillis (0);
        //
     //   int riserZero = robot.pulleyDrive.getCurrentPosition();

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
                // We want to READ the Encoders here
                //    ONLY set the motors in motion in ONE place.
             //   rightMotorPos = robot.rightDrive.getCurrentPosition();
              //  lefMotorPos = robot.leftDrive.getCurrentPosition();
               // riserMotorPos = robot.pulleyDrive.getCurrentPosition();

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
                // init drive min and max to default values.  We'll reset them to other numbers
                // if conditions demand it.
                stageTimer += NAVPERIOD;

                switch ( currState) {
                    case 0: // move the ball
                        break;
                    case 1:  //Close clamp on cube
                        //leftClamp_Cmd = robot.LEFTCLAMPED;
                        //rightClamp_Cmd = robot.RIGHTCLAMPED;
                        break;
                    case 2:   //Lift cube small amount
                        riserCmd = riserMax;
                        break;
                    case 3:   // Drive forward towards cryptobox
                        leftDriveCmd = driveMax;
                        rightDriveCmd = driveMax;
                        leftRearCmd = driveMax;
                        rightRearCmd = driveMax;
                        break;
                    case 4: // crab .. left for red, right for blue
                        leftDriveCmd = driveMax;
                        rightDriveCmd = (float)-1*driveMax;
                        leftRearCmd = (float)-1*driveMax;
                        rightRearCmd = driveMax;
                        break;
                    case 5: // fwd
                        leftDriveCmd = (float)0.50;
                        rightDriveCmd = (float)0.50;
                        leftRearCmd = (float)0.50;
                        rightRearCmd = (float)0.50;
                        break;
                    case 6: // unclamp
                       // leftClamp_Cmd = robot.LEFTUNCLAMPED;
                        //rightClamp_Cmd = robot.RIGHTUNCLAMPED;
                        break;
                    case 7: //reverse
                        leftDriveCmd = (float)-0.50;
                        rightDriveCmd = (float)-0.50;
                        leftRearCmd = (float)-0.50;
                        rightRearCmd = (float)-0.50;
                        break;
                    case 8:
                        stageTimer = 0;
                        break;
                    default:
                        break;

                }
                if (stageTimer > timeLimit[currState]) {
                    riserCmd = 0;
                    leftDriveCmd=0;
                    rightDriveCmd=0;
                    leftRearCmd=0;
                    rightRearCmd=0;
                    stageTimer= 0;

                    currState++;
                }
                // mapping inputs to servo command

                // The ONLY place we set the motor power request. Set them here, and
                // we will never have to worry about which set is clobbering the other.

                // Servo commands: Clipped and Clamped.

                // motor commands: Clipped & clamped.
                leftDriveCmd  = Range.clip((float)leftDriveCmd,      driveMin, driveMax);
                rightDriveCmd = Range.clip((float)rightDriveCmd,     driveMin, driveMax);
                leftRearCmd   = Range.clip((float)leftRearCmd,       driveMin, driveMax);
                rightRearCmd  = Range.clip((float)rightRearCmd,   driveMin, driveMax);
                riserCmd      = Range.clip((float)riserCmd, riserMin, riserMax);
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
               // robot.leftClamp.setPosition(leftClamp_Cmd);
               // robot.rightClamp.setPosition(rightClamp_Cmd);
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
               // robot.leftDrive.setPower(leftDriveCmd);

                /*  Left Rear Motor Power  */
              //  robot.leftRear.setPower(leftRearCmd);

                /*  Right Drive Motor Power */
              //  robot.rightDrive.setPower(rightDriveCmd);

                /*Right Rear Motor Power*/
               // robot.rightRear.setPower(rightRearCmd);
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
                telemetry.addData("Switch State ", currState);
                telemetry.addData("Switch Timer ", stageTimer );
                telemetry.update();
            }
        }
        telemetry.addData("Path", "Complete");
        telemetry.update();
    }
}