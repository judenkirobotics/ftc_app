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
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//@Autonomous(name="Pushbot: Auto Drive By Encoder", group="Pushbot")

@Autonomous(name="Drive Test", group="Pushbot")
@SuppressWarnings("WeakerAccess")
//@TeleOp(name = "Time Slice Op Mode", group = "HardwarePushbot")
//@Disabled
public class JK_DriveTest extends LinearOpMode {

    JK_19_HardwarePushbot robot   = new JK_19_HardwarePushbot();   // Use a Pushbot's hardware
    Drive robotDrive = new Drive();
    final long SENSORPERIOD = 20;
    final long ENCODERPERIOD = 20;
    final long SERVOPERIOD = 20;
    final long NAVPERIOD = 20;
    final long MOTORPERIOD = 20;
    final long CONTROLLERPERIOD = 20;
    final long TELEMETRYPERIOD = 500;

    float stageTime = 0;
    final int FWD = 0;
    final int REV = 1;
    final int TRN1 = 2;
    final int TRN2 = 3;
    final int TRN3 = 4;
    final int FUNKY = 5;
    final int WAIT = 6; // MUST be highest numbered state!!
    final float MAX_RED_GREEN = (float)1.75;
    final float MIN_RED_GREEN = (float)1.2;
    final float MAX_BLUE_GREEN = (float)0.65;
    final float MIN_BLUE_GREEN = (float)0.0;

    int[] stage =       {FUNKY, FWD,  REV,  TRN1, TRN2, TRN3, WAIT};
    double[] stageLim = {20000, 5000, 5000, 5000, 5000, 5000, 5000};

    int CurrentAutoState = 0;



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
        long LastServo = CurrentTime + 10;
        long LastNav = CurrentTime + 15;
        long LastMotor = CurrentTime + 20;
        long LastTelemetry = CurrentTime + 17;



        //Configure the drivetrain
        Drive.Parameters dParm = robotDrive.getParameters();
        dParm.frontRight = robot.rightDrive;
        dParm.frontLeft  = robot.leftDrive;
        dParm.rearRight  = robot.rightDrive;  //Should be unnecessary, but just keep the nulls away
        dParm.rearLeft   = robot.leftDrive;
        dParm.driveType  = Drive.DriveType.TANK;
        dParm.imu        = robot.imu;
        dParm.motorRatio = 28;
        dParm.gearRatio  = 40;
        dParm.wheelDiameter  = 2.5;
        dParm.mecanumAngle   = 1;
        dParm.pivotTolerance  = Drive.PivotTolerance.THREE_DEGREES;
        dParm.encoderTolerance = 15;   //VERY DANGEROUS TO PLAY WITH CAN RESULT IN STUCK STATE
        dParm.turnBackoff = 0.06;  // Six percent backoff
        dParm.backoffMultiplier = 3;
        dParm.opMode      = this;
        dParm.debug       = true;
        robotDrive.configureDrive(dParm);



        telemetry.addData("Status   ", "Initialized");
        telemetry.update();

        ElapsedTime runtime = new ElapsedTime();


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




                switch (stage[CurrentAutoState]) {
                    case FWD:
                        if (robotDrive.getMoveStatus() == Drive.MoveStatus.AVAILABLE) {
                            robotDrive.move(Drive.MoveType.FORWARD, 18, 0.3);
                        }
                        if (robotDrive.getMoveStatus() == Drive.MoveStatus.COMPLETE) {
                            robotDrive.move(Drive.MoveType.STOP, 0, 0);
                            stage_complete = true;
                        }
                        break;
                    case REV:
                        if (robotDrive.getMoveStatus() == Drive.MoveStatus.AVAILABLE) {
                            robotDrive.move(Drive.MoveType.REVERSE, 18, 0.6);
                        }
                        if (robotDrive.getMoveStatus() == Drive.MoveStatus.COMPLETE) {
                            robotDrive.move(Drive.MoveType.STOP, 0, 0);
                            stage_complete = true;
                        }
                        break;
                    case TRN1:
                        if (robotDrive.getMoveStatus() == Drive.MoveStatus.AVAILABLE) {
                            robotDrive.move(Drive.MoveType.PIVOTRIGHT, 90, 0.45);
                        }
                        if (robotDrive.getMoveStatus() == Drive.MoveStatus.COMPLETE) {
                            robotDrive.move(Drive.MoveType.STOP, 0, 0);
                            stage_complete = true;
                        }
                        break;
                    case TRN2:
                        if (robotDrive.getMoveStatus() == Drive.MoveStatus.AVAILABLE) {
                            robotDrive.move(Drive.MoveType.PIVOTLEFT, 270, 0.45);
                        }
                        if (robotDrive.getMoveStatus() == Drive.MoveStatus.COMPLETE) {
                            robotDrive.move(Drive.MoveType.STOP, 0, 0);
                            stage_complete = true;
                        }
                        break;
                    case TRN3:
                        if (robotDrive.getMoveStatus() == Drive.MoveStatus.AVAILABLE) {
                            robotDrive.move(Drive.MoveType.PIVOTRIGHT, 0, 0.45);
                        }
                        if (robotDrive.getMoveStatus() == Drive.MoveStatus.COMPLETE) {
                            robotDrive.move(Drive.MoveType.STOP, 0, 0);
                            stage_complete = true;
                        }
                        break;
                    case FUNKY:
                    {
                        if (robotDrive.getMoveStatus() == Drive.MoveStatus.AVAILABLE) {
                            robotDrive.move(Drive.MoveType.RIGHTFORWARD, 12, 0.6);
                        }
                        if (robotDrive.getMoveStatus() == Drive.MoveStatus.COMPLETE) {
                            robotDrive.move(Drive.MoveType.STOP, 0, 0);
                            stage_complete = true;
                        }
                        break;
                    }
                    case WAIT:
                        break;
                    default:
                        break;
                }
                if ((stageTime >= stageLim[CurrentAutoState]) ||
                        (stage_complete)){
                    stageTime = 0;

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

                robotDrive.update();

            }

            /* ***************************************************
             *                SERVO OUTPUT
             *       Inputs:  Motor power commands
             *       Outputs: Physical interface to the servos
             ****************************************************/
            if (CurrentTime - LastServo > SERVOPERIOD) {
                LastServo = CurrentTime;

            }


            /* ***************************************************
             *                TELEMETRY
             *       Inputs:  telemetry structure
             *       Outputs: command telemetry output to phone
             ****************************************************/
            if (CurrentTime - LastTelemetry > TELEMETRYPERIOD) {
                LastTelemetry = CurrentTime;
               telemetry.addData("Current index: ", CurrentAutoState);
               telemetry.addData("Current State: ", stage[CurrentAutoState]);
               telemetry.setAutoClear(true);
               telemetry.update();
            }
        } // end of while opmode is active

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

}