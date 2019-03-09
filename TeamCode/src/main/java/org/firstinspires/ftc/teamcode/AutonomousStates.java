package org.firstinspires.ftc.teamcode;


/*
 * Created for Juden Ki 8578 and Kernel Panic 11959
 *
 * October 2016
 * Initial Concept by Howard Bishop (mentor)
 * First Coding by Jeffrey and Alexis
 *
 * October 2017
 * Second Coding by Katherine amd Jeffrey
 *
 * December 2018
 * Major overhaul into single state engine fed an array of commands by Jim Rumsey (mentor)
 *
 *  OVERVIEW
 *  Generic autonomous file that should be unique to each robot.   It is fine to copy and rename
 *  this class.   Just have the autonomous runmodes target the copy.   The MOVE, PAUSE, and WAIT
 *  states should be generic and common to all robots.   All other states are robot specific and
 *  should map directly to a task that the robot is supposed to accomplish.   Push a button, find
 *  something of a particular color, etc.....
 *
 *  REQUIRED
 *  REV Robotics Expansion Hub --  Due to IMU the only supported hub
 *  IMU                        --  Rev IMU configured as Adafruit IMU on I2C 0
 *  xxxHardwarePushbot         --  Robot class that MUST define and initialize all of the hardware
 *                                 that will be used in autonomous code.
 *
 *  FIRST TIME ON NEW ROBOT
 *      1.  Prune the AutoStates enum and the corresponding case statement to remove references to
 *          hardware/states that do not exists for the new robot.
 *      2.  Configure all of the Drive parameters for the robot  (dParm).  For a fuller explanation
 *          of these parameters refer to the Drive.java Class.   NOTE:  The Drive class is unitless
 *          for distance measurements.  The wheel diameter just needs to be configured int the
 *          desired units.    (Ex:  2.5 if inches, or 6.35 if cm)
 *
 *  CREATE NEW STATE
 *
 *      1.  Add an entry to the enum AutoStates.  This should be a name that describes the purpose
 *          of this new state.   (Ex: FLIP -- flips out a robot arm.)
 *      2.  Go the the NAVIGATION section of this class.   Add the new state to the case
 *          statement.   Add the code that will perform the purpose of this state.  Take care to
 *          avoid while loops when creating this code.  Nothing should be done that will prevent the
 *          robot from responding to the STOP button on the driver station
 *      3.  Test the new state.  This can be done by making the new state the first entry on
 *          AutoCommand array that is passed to this class.  (See JK_AutonomousExample.java)  Take
 *          care to verify that all value(1-4) inputs produce the desired behaviour.
 *      4.  MOST IMPORTANT -- Once the new state has been created update the state table in the
 *          comments below.  Remember if it is not documented it does not exist!
 *
 *
 ***************************************************************************************************
 * STATE   MOVEMENT          Description and Values 1-3
 ***************************************************************************************************
 * MOVE    FORWARD             Moves robot forward value1 units at speed value2
 *         REVERSE             Moves robot reverse value1 units at speed value2
 *         CRABLEFT            Crabs robot left value1 units at speed value2 (mecanum only)
 *         CRABRIGHT           Crabs robot right value1 units at speed value2 (mecanum only)
 *         PIVOTLEFT           Turns robot to the left to heading value1 at speed value2
 *         PIVOTRIGHT          Turns robot to the right to heading value1 at speed value2
 *         LEFTFORWARD         Engages left side drive to move forward value1 units at speed value2
 *         LEFTREVERSE         Engages left side drive to move reverse value1 units at speed value2
 *         RIGHTFORWARD        Engages right side drive to move forward value1 units at speed value2
 *         RIGHTREVERSE        Engages right side drive to move reverse value1 units at speed value2
 *         STOP                Tells robot to stop moving
 *
 * SWING   N/A                  Moves color sensor arm to servo position value1
 *
 * DETECT  N/A                  Swings color sensor arm looking for gold mineral.  If mineral is
 *                              found stops swing and actuates the CR servo paddle.   Starts color
 *                              sensor arm at position value1 with a transit increment of value2.
 *                              Actuates paddle at value3 power for value4 time.  Sets internal
 *                              flag if gold mineral found.
 *
 * KICK                         Swings the paddle at power value1 for time value2 only if the gold
 *                              mineral has not been found.
 *
 * PADDLE  N/A                  Swing the paddle CR servo value1   (REV -1.0 to 1.0 FWD)
 *
 * PAUSE   N/A                  Essentially a sleep state for robot, but not final wait state
 *
 * WAIT    N/A                  Final state should be last called.   (Not strictly necessary)
 *
 */




import android.os.SystemClock;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;




public class AutonomousStates {
    //Define Robot Hardware and classes here
    private JK_19_HardwarePushbot robot   = new JK_19_HardwarePushbot();   //Remap to your robot
    private Drive robotDrive              = new Drive();
    private GoldBlockDetection blockDetection = new GoldBlockDetection();


    //Define all of the available states for AutoState.   Add new states before PAUSE
    public enum AutoStates {LOWER, DETECTGOLD, PUSHBLOCK, BLOCKPIVOT, BLOCKFORWARD, BLOCKREVERSE ,MOVE, SWING, PADDLE, DETECT, TOKEN, PAUSE, WAIT, RAISE}


    //Define AutoState run intervals here
    private final long SENSORPERIOD     = 20;
    private final long SERVOPERIOD      = 20;
    private final long NAVPERIOD        = 20;
    private final long MOTORPERIOD      = 20;
    private final long TELEMETRYPERIOD  = 500;
    private float stageTime             = 0;
    int CurrentAutoState                = 0;
    static final int   THIRTY_SECONDS   = 30 * 1000;

    //Robot specific variables
    double armPosition   = 0;
    double paddlePower   = 0;
    double colorSensorR = 0.0;
    double colorSensorG = 0.0;
    double colorSensorB = 0.0;
    boolean isGold = false;
    final double MAX_RED_GREEN = 1.75;
    final double MIN_RED_GREEN = 1.2;
    final double MAX_BLUE_GREEN = 0.65;
    boolean paddleStarted = false;
    boolean goldHasBeenFound = false;

    private int liftCurrent = 0;
    private int liftMotorCounts = 28;
    private int liftGearCounts = 20;
    private int liftThreadPitch = 2;
    private int liftStartsPerRev = 4;
    private double liftScrewLength = 187;
    private int liftCountsPerRev = liftMotorCounts * liftGearCounts;
    private int liftVertPerRev   =  liftThreadPitch * liftStartsPerRev;
    private double liftFullExtensionRev = (double)liftScrewLength / (double) liftVertPerRev;
    private double liftFullExtensionCount = liftCountsPerRev * liftFullExtensionRev;
    private boolean liftStarted = false;
    private double liftPower = 0.0;
    private int liftTarget = 0;

    double blockHeading = 0.0;
    double blockDistance = 0.0;


    public void runOpMode(LinearOpMode opMode, HardwareMap hardwareMap, AutoCommand cmd[]) {

        /*
         * Initialize all of the robot hardware.
         * The init() method of the hardware class does all the work here
         */
        opMode.telemetry.addData("Status", "Initializing Robot...");
        opMode.telemetry.addData("Status", "Configuring Hardware...");
        opMode.telemetry.setAutoClear(false);
        opMode.telemetry.update();
        robot.init(hardwareMap, true);
        if (robot.imu.isGyroCalibrated()) {
            opMode.telemetry.addLine("    Gyro Calibrated");
        }
        else {
            opMode.telemetry.addLine("    Gyro FAILED CALIBRATION");
        }
        opMode.telemetry.addData("      ", "SUCCESS!");
        opMode.telemetry.update();

        opMode.telemetry.addLine("Initializing Vision");
        opMode.telemetry.update();
        blockDetection.configureDetection(opMode);




        /*****************************************************
         *  CONFIGURE THE DRIVE TRAIN  THIS IS ROBOT SPECIFIC
         *****************************************************/
        Drive.Parameters dParm = robotDrive.getParameters();
        dParm.frontRight       = robot.rightDrive;
        dParm.frontLeft        = robot.leftDrive;
        dParm.rearRight        = robot.rightDrive;  //Should be unnecessary, but just keep the nulls away
        dParm.rearLeft         = robot.leftDrive;
        dParm.frPolarity       = DcMotorSimple.Direction.FORWARD;
        dParm.rrPolarity       = DcMotorSimple.Direction.FORWARD;
        dParm.flPolarity       = DcMotorSimple.Direction.REVERSE;
        dParm.rlPolarity       = DcMotorSimple.Direction.REVERSE;
        dParm.driveType        = Drive.DriveType.TANK;
        dParm.imu              = robot.imu;
        dParm.motorRatio       = 28;
        dParm.gearRatio        = 40;
        dParm.wheelDiameter    = 2.5;
        dParm.mecanumAngle     = 1;
        dParm.pivotTolerance   = Drive.PivotTolerance.ONE_DEGREE;
        dParm.encoderTolerance = 75;    //VERY DANGEROUS TO PLAY WITH CAN RESULT IN STUCK STATE
        dParm.turnBackoff      = 0.45;  // 45 percent backoff
        dParm.backoffMultiplier = 15;    // Make larger for high speed turns.
        dParm.minStartPower    = 0.45;
        dParm.minTurnPower     = 0.45;
        dParm.opMode           = opMode;
        dParm.debug            = false;
        dParm.useEncoderRatio  = true;
        dParm.Block_Location   = GoldBlockDetection.LOCATION.ERROR;
        if (robotDrive.configureDrive(dParm)) {
            opMode.telemetry.addData("Status   ", "Robot Initialized!");
        }
        else {
            opMode.telemetry.addData("Status   ", "INITIALIZATION FAILED!!!!!");
        }


        /*
         * Perform a little range checking on the supplied array of states.  Look to see if
         * the autonomous time limit will be exceeded
         */
        int    i =0,t = 0;
        while (i < cmd.length) {
            t += cmd[i].timeLimit;
            i++;
        }
        if (t>THIRTY_SECONDS) {
            opMode.telemetry.addLine("WARNING... Autonomous Commands may exceed time");
            opMode.telemetry.addData("   Allowed", THIRTY_SECONDS);
            opMode.telemetry.addData("   Actual ", t);
        }
        opMode.telemetry.update();
        opMode.telemetry.setAutoClear(true);


        /*
         * Define and initialize all of the loop timing variables
         */
        long CurrentTime    = System.currentTimeMillis();
        long LastSensor     = CurrentTime;
        long LastServo      = CurrentTime + 10;
        long LastNav        = CurrentTime + 15;
        long LastMotor      = CurrentTime + 20;
        long LastTelemetry  = CurrentTime + 17;


        ElapsedTime runtime = new ElapsedTime();
        opMode.waitForStart();
        runtime.reset();

        /* ************************************************************
         *            Everything below here runs after START          *
         **************************************************************/
        // run until the end of the match (driver presses STOP)
        while (opMode.opModeIsActive()) {
            CurrentTime = System.currentTimeMillis();


            /* *******************************************************************
             *                SENSORS
             *        Inputs:  Sensor Values from robot (not drive motor encoders)
             *        OUTPUTS: parameters containing sensor values
             *
             *********************************************************************/
            if (CurrentTime - LastSensor > SENSORPERIOD) {
                LastSensor = CurrentTime;
                liftCurrent = robot.LiftMotor.getCurrentPosition();
            }


            /*****************************************************************
             *                NAV
             *      Inputs:  Sensor Values (as needed)
             *               Motor Encoders (accessed through Drive()
             *               IMU Heading    (accessed through Drive()
             *      Outputs: Servo and non drive train Motor commands
             *               drive train Motor commands stored in Drive() class
             ******************************************************************/
            if (CurrentTime - LastNav > NAVPERIOD) {
                LastNav = CurrentTime;
                boolean stage_complete = false;
                stageTime += NAVPERIOD;

                switch (cmd[CurrentAutoState].state) {
                    case MOVE:
                        if (robotDrive.getMoveStatus() == Drive.MoveStatus.AVAILABLE) {
                            robotDrive.move(cmd[CurrentAutoState].moveType, (int)cmd[CurrentAutoState].value1,cmd[CurrentAutoState].value2);
                        }
                        if (robotDrive.getMoveStatus() == Drive.MoveStatus.COMPLETE) {
                            robotDrive.move(Drive.MoveType.STOP, 0, 0);
                            stage_complete = true;
                        }
                        break;
                    case SWING:
                        // IDEA: disregard following code in SWING and PADDLE. combine swing and detection states. write a "detectGold()" function. construct an array of positions
                        //to detect gold at. if gold is detected, stage_complete. if gold not detected, search next position.
                        armPosition = cmd[CurrentAutoState].value1;
                        break;
                    case PADDLE:
                        if(isGold  || paddleStarted) {
                            paddlePower = cmd[CurrentAutoState].value1;
                            paddleStarted = true;
                            isGold = false;
                        } //the logic here? how to differentiate when gold is detected and when it is not? how will autonomous states combine it with swing?
                        break;
                    case DETECT:
                        //NEED TO PULL IN JK CODE FROM SHOP!!!
                        // some ideas: detect just detects the color. get color sensor values and compare with thresholds.
                        colorSensorR = (double) robot.MineralColorSensor.red();
                        colorSensorG = (double) robot.MineralColorSensor.green();
                        colorSensorB = (double) robot.MineralColorSensor.blue();
                        isGold = false; //default value
                        //opMode.telemetry.clear();
                        //opMode.telemetry.addData("Green ", colorSensorG);
                        //opMode.telemetry.addData("Red ", colorSensorR);
                        //opMode.telemetry.addData("Blue", colorSensorB);
                        //opMode.telemetry.addData( "R/G ", (colorSensorR/colorSensorG));
                        //opMode.telemetry.addData( "B/G ", (colorSensorB/colorSensorG));
                        //opMode.telemetry.addData("MAX_RED_GREEN ", MAX_RED_GREEN);
                        //opMode.telemetry.addData("MIN_RED_GREEN ", MIN_RED_GREEN);
                        //opMode.telemetry.addData("MAX_BLUE_GREEN ", MAX_BLUE_GREEN);
                        //opMode.telemetry.update();
                        //opMode.sleep(5000);
                        if (colorSensorG > 0){
                            if ((colorSensorR/colorSensorG < MAX_RED_GREEN) &&
                                    (colorSensorR/colorSensorG > MIN_RED_GREEN) &&
                                    (colorSensorB/colorSensorG < MAX_BLUE_GREEN)){
                                isGold = true; // changes to true only if the if statement is satisfied
                                stage_complete = true;  // Only complete stage if gold is found, otherwise timeout????
                                goldHasBeenFound = true;
                            }
                        }
                        break;
                    case DETECTGOLD:
                        //GoldBlockDetection.LOCATION location;
                        dParm.Block_Location = blockDetection.detectGoldBlock(cmd[CurrentAutoState].timeLimit);
                        opMode.telemetry.addData("block located ",dParm.Block_Location );
                        opMode.telemetry.update();
                        stage_complete = true;
                        break;
                    case PUSHBLOCK:
                        stage_complete = true;
                        switch(dParm.Block_Location) {
                            case LEFT:
                                blockHeading = 326;
                                blockDistance = 17;
                            case CENTER:
                                blockHeading = 0;
                                blockDistance = 13;
                            case RIGHT:
                                blockHeading = 30;
                                blockDistance = 20;
                            case ERROR:
                            default:
                                break;
                        }
                        // have robot turn to block heading, move forward block distance, move backwards block distance
                        break;
                    case BLOCKPIVOT:
                        if (robotDrive.getMoveStatus() == Drive.MoveStatus.AVAILABLE) {
                            Drive.MoveType type = Drive.MoveType.STOP;
                            if (dParm.Block_Location == GoldBlockDetection.LOCATION.LEFT) {
                                type = Drive.MoveType.PIVOTLEFT;
                            }
                            else if (dParm.Block_Location == GoldBlockDetection.LOCATION.RIGHT) {
                                type = Drive.MoveType.PIVOTRIGHT;
                            }
                            else {
                                stage_complete = true;
                            }
                            if (!stage_complete) {
                                robotDrive.move(type , blockHeading, 0.5);
                            }

                        }
                        if (robotDrive.getMoveStatus() == Drive.MoveStatus.COMPLETE) {
                            robotDrive.move(Drive.MoveType.STOP, 0, 0);
                            stage_complete = true;
                        }
                        break;
                    case BLOCKFORWARD:
                        if (robotDrive.getMoveStatus() == Drive.MoveStatus.AVAILABLE) {
                            blockDistance += cmd[CurrentAutoState].value3;
                            robotDrive.move(Drive.MoveType.FORWARD, blockDistance, 0.7);
                        }
                        if (robotDrive.getMoveStatus() == Drive.MoveStatus.COMPLETE) {
                            robotDrive.move(Drive.MoveType.STOP, 0, 0);
                            stage_complete = true;
                        }
                        break;
                    case BLOCKREVERSE:
                        if (robotDrive.getMoveStatus() == Drive.MoveStatus.AVAILABLE) {
                            blockDistance -= cmd[CurrentAutoState].value3;
                            robotDrive.move(Drive.MoveType.REVERSE, blockDistance, 0.7);
                        }
                        if (robotDrive.getMoveStatus() == Drive.MoveStatus.COMPLETE) {
                            robotDrive.move(Drive.MoveType.STOP, 0, 0);
                            stage_complete = true;
                        }
                        break;
                    case TOKEN:
                        robot.FlipServo.setPosition(cmd[CurrentAutoState].value1);
                        break;
                    case PAUSE:
                        break;
                    case WAIT:
                        break;
                    case LOWER:
                        int targetPosition = (int) (liftFullExtensionCount * cmd[CurrentAutoState].value1);
                        if (!liftStarted) {
                            robot.LiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            robot.LiftMotor.setTargetPosition(targetPosition);
                            liftStarted = true;
                            liftTarget = targetPosition;
                            liftPower = 1.0;
                        }
                        if ((Math.abs (liftCurrent - targetPosition) < (liftCountsPerRev/4)) ||
                                (liftCurrent > targetPosition) ) {
                            liftStarted = false;
                            liftPower = 0.0;
                            stage_complete = true;
                        }

                        break;
                    case RAISE:
                        if (!liftStarted) {
                            robot.LiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            robot.LiftMotor.setTargetPosition(0);
                            liftStarted = true;
                            liftTarget = 0;
                            liftPower = 1.0;
                        }
                        if (Math.abs (liftCurrent - 0) < (liftCountsPerRev/4)) {
                            liftStarted = false;
                            liftPower = 0.0;
                            stage_complete = true;
                        }
                        break;
                    default:
                        robotDrive.move(Drive.MoveType.STOP, 0, 0);
                        break;
                }

                /*
                 * Check to see if there is another state to run, Reset stage time, possibly
                 * update/clear local parameters if required (robot specific)
                 */
                if ((stageTime >= cmd[CurrentAutoState].timeLimit) || (stage_complete)){
                    stageTime = 0;
                    paddlePower = 0;  //CR Servo
                    paddleStarted = false;
                    //In case a move or turn times out.
                    robotDrive.move(Drive.MoveType.STOP, 0,0);
                    if (CurrentAutoState  < (cmd.length - 1)) {
                        CurrentAutoState ++;
                    }
                }
            }


            /* ***************************************************
             *                MOTOR OUTPUT
             *       Inputs:  Motor power commands
             *       Outputs: Physical interface to the motors
             ****************************************************/
            if (CurrentTime - LastMotor > MOTORPERIOD) {
                LastMotor = CurrentTime;

                //Have the Drive() class run an update on all of the drive train motors
                robotDrive.update();
                if ((liftStarted) && (Math.abs(liftCurrent - liftTarget) > liftCountsPerRev/4)) {
                    robot.LiftMotor.setPower(liftPower);
                }
                else {
                    liftPower = 0.0;
                    robot.LiftMotor.setPower(liftPower);
                }

            }

            /* ***************************************************
             *                SERVO OUTPUT
             *       Inputs:  Motor power commands
             *       Outputs: Physical interface to the servos
             ****************************************************/
            if (CurrentTime - LastServo > SERVOPERIOD) {
                LastServo = CurrentTime;
                robot.ColorSensingServo.setPosition(armPosition);
                robot.PaddleServo.setPower(paddlePower);
            }


            /* ***************************************************
             *                TELEMETRY
             *       Inputs:  telemetry structure
             *       Outputs: command telemetry output to phone
             ****************************************************/
            if (CurrentTime - LastTelemetry > TELEMETRYPERIOD) {
                LastTelemetry = CurrentTime;
                opMode.telemetry.addData("Current index: ", CurrentAutoState);
                opMode.telemetry.addData("Current State: ", cmd[CurrentAutoState].state);
                opMode.telemetry.addData("Current Type : ", cmd[CurrentAutoState].moveType);
                opMode.telemetry.addData("Time Limit   : ", cmd[CurrentAutoState].timeLimit);
                opMode.telemetry.addData("Value 1      : ", cmd[CurrentAutoState].value1);
                opMode.telemetry.addData("Value 2      : ", cmd[CurrentAutoState].value2);
                opMode.telemetry.addData("Value 3      : ", cmd[CurrentAutoState].value3);
                opMode.telemetry.update();
            }
        } // end of while opmode is active

        opMode.telemetry.addData("Path", "Complete");
        opMode.telemetry.update();
    }

}
