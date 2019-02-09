/*
Built for Juden Ki 8578 and Kernel Panic 11959

James K Rumsey
12/18/2018

OVERVIEW:
Provides methods to autonomously move the robot.    Includes forward, reverse, pivots, and crab.
To use this class it must first be configured.
        getParameters()
        configureDrive()
Then movements can be initiated
        move()
Once initiated the update routine performs all of the monitoring.
        update()
This means the user must monitor if the requested move is complete.  This gives the user the ability
to have other logic running while the movements are taking place.   This also allows the user to
interrupt a movement.   Refer to the sample autonomous software JK_DriveTest for an example on how
to use this class.


REQUIRED:
Gyro -- Supports REV IMU gyro
Encoders -- Encoder cables to all drive motors are required, this means for for mecanum drive.
Motors -- Encoder capable motors, all motors identical, asymmetric drive trains are not supported.
                Tank drive trains currently only support two motors
Drive Train -- Drive train should be tight and responsive. Variable drag, variable traction,
               asymmetry in treads, will all result in less accurate results.
UNITS  --  Angles are measured in degrees the 'forward' direction of the robot when initialized
                 is 0 degrees.   Pivoting right will increase up till 360 is reached then wrap to
                 0.   Radians are not currently supported.
           Distance is unit less.   Just be consistent on the desired distance units and the wheel
                 diameter units.


PARAMETERS:
    DcMotor        frontRight     -- Front right motor on robot for mecanum, or right for tank.
    DcMotor        frontLeft
    DcMotor        rearRight
    DcMotor        rearLeft
    DcMotor.Direction frPolarity  -- FORWARD(default) or REVERSE depending on motor orientation.
    DcMotor.Direction flPolarity  -- REVERSE(default)
    DcMotor.Direction rrPolarity  -- FORWARD(default)
    DcMotor.Direction rlPolarity  -- REVERSE(default)
    BNO055IMU      imu            -- Inertial management unit for
    DriveType      driveType      -- Either TANK or MECANUM
    double         motorRatio     -- This is the number of counts for a single rotation of the motor
                                     shaft.
    double         gearRatio      -- This is the ratio on the gearbox.  If no gearbox is present
                                     set this value to 1.
    double         wheelDiameter  -- The diameter of the wheel attached to the motor shaft.  All
                                     DRIVE wheels MUST be the same diameter.  PASSIVE wheels can be
                                     any size.
    double         mecanumAngle   -- The angle of the mecanum roller relative to the axle through
                                     the front right wheel.  Most are 45.
    PivotTolerance pivotTolerance -- The margin of error allowed on a pivot.   If the tolerance is
                                     set to ONE_DEGREE and the pivot target is 90 then a pivot of 89
                                     to 91 will be considered successful.
    double         turnBackoff    -- Expressed as a percentage 0.0 to 1.0 the amount of power the
                                     pivot is reduced by as it nears completion of the pivot.  This
                                     power reduction occurs inside the window created by the
                                     backoffMultiplier * pivotTolerance.    A backoff of 3 and a
                                     tolerance of 2 would start power reduction to the motors when
                                     within 6 degrees of the pivot target.   WARNING:  The four
                                     pivot parameters will be extremely robot dependent.   Robot
                                     weight (momentum), wheel/tread traction, and motor braking
                                     force all impact accurate turns.
    double         backoffMultiplier -- Scaler used to determine backoff window
    int            encoderTolerance  -- used to determine the completion window for encoder based
                                     movements.   Minimum value is half the motorRatio.   This may
                                     still be too small for larger/heavier robots.
    double         minStartPower     --  The minimum amount of power that will be supplied to the
                                     motors when starting a maneuver regardless of specified power.
    double         minTurnPower      --  The minimum amount of power that will be supplied to the
                                     motors when finishing a pivot regardless of backoff.  Balance
                                     this
    LinearOpMode   opMode            -- the telop/autonomous program running. Should always be 'this'
    boolean        debug             --  If set to true will send messages to telemetry.



STILL TO DO:
    Add drift correction to FORWARD and REVERSE
    Verify Mecanum works for FORWARD, REVERSE, PIVOTLEFT, and PIVOTRIGHT
    Debug  CRABLEFT and CRABRIGHT
    Add multi motor support for tank drive??  Afe to do this??
    Maybe eliminate some of the tank/mecanum checks.   Could just double set motors a lot of the
        time with no negative results.  Code is slightly less efficient but could condense it
        significantly.
    Add support for old school gyro??  Add support for dual gyro / sanity check??
    Create a telop() mode that can be used to 'discover' best parameter settings
    Any reason to allow multiple initialPowers?   Perhaps for more complex movements.


*/
package org.firstinspires.ftc.teamcode;



import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.sun.tools.javac.tree.DCTree;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


public class Drive {

    public enum DriveType  {TANK, MECANUM;}
    public enum MoveType   {FORWARD, REVERSE, CRABLEFT, CRABRIGHT, PIVOTLEFT, PIVOTRIGHT, LEFTFORWARD, LEFTREVERSE, RIGHTFORWARD, RIGHTREVERSE, STOP;}
    public enum MoveStatus {INITIATED, INPROGRESS, COMPLETE, AVAILABLE;}
    public enum  PivotTolerance {ZERO_DEGREES, ONE_DEGREE, TWO_DEGREES, THREE_DEGREES, FOUR_DEGREES, FIVE_DEGREES;}



    //NOTE:   Set all of the values to "known good" values except for the motors, imu, and opMode
    public class Parameters {
        public DcMotor        frontRight     = null;
        public DcMotor        frontLeft      = null;
        public DcMotor        rearRight      = null;
        public DcMotor        rearLeft       = null;
        public DcMotor.Direction frPolarity  = DcMotor.Direction.FORWARD;
        public DcMotor.Direction flPolarity  = DcMotor.Direction.REVERSE;
        public DcMotor.Direction rrPolarity  = DcMotor.Direction.FORWARD;
        public DcMotor.Direction rlPolarity  = DcMotor.Direction.REVERSE;
        public BNO055IMU      imu            = null;
        public DriveType      driveType      = DriveType.TANK;
        public double         motorRatio     = 28;
        public double         gearRatio      = 40;
        public double         wheelDiameter  = 2.5;
        public double         mecanumAngle   = 45;
        public PivotTolerance pivotTolerance = PivotTolerance.ONE_DEGREE;
        public double         turnBackoff    = 0.05;
        public double         backoffMultiplier = 3;
        public int            encoderTolerance = 15;
        public double         minStartPower  = 0.3;
        public double         minTurnPower   = 0.3;
        public LinearOpMode   opMode         = null;
        public boolean        debug          = false;
    }

    private class Maneuver {
        private MoveType  type;
        private int       rightTarget;
        private int       leftTarget;
        private int       rearRightTarget;
        private int       rearLeftTarget;
        private float     angleTarget;
        private double    initialPower;
        private MoveStatus status            = MoveStatus.COMPLETE;
        private double    rfPower;
        private double    lfPower;
        private double    rrPower;
        private double    lrPower;
    }

    private boolean     dConfigured       = false;
    private boolean     backoffStarted    = false;
    private Parameters  par               = new Parameters();
    private Maneuver    maneuver          = new Maneuver();
    private double      circumference     = 0;
    private double      driveRatio        = 0;
    private double      crabRatio         = 0;





    public Parameters getParameters () {
        return (par);
    }




    //
    // Perform some basic sanity checking on the calibration data.
    //
    public boolean configureDrive (Parameters parameters)
    {
        dConfigured = true;

        // Java Kung Fu is weak should be a way to do this without the copy
        par.frontRight    = parameters.frontRight;
        par.frontLeft     = parameters.frontLeft;
        par.rearRight     = parameters.rearRight;
        par.rearLeft      = parameters.rearLeft;
        par.frPolarity    = parameters.frPolarity;
        par.rrPolarity    = parameters.rrPolarity;
        par.flPolarity    = parameters.flPolarity;
        par.rlPolarity    = parameters.rlPolarity;
        par.driveType     = parameters.driveType;
        par.imu           = parameters.imu;
        par.motorRatio    = parameters.motorRatio;
        par.gearRatio     = parameters.gearRatio;
        par.wheelDiameter = parameters.wheelDiameter;
        par.mecanumAngle  = parameters.mecanumAngle;
        par.pivotTolerance     = parameters.pivotTolerance;
        par.debug              = parameters.debug;
        par.turnBackoff        = 1 - parameters.turnBackoff;
        par.encoderTolerance   = parameters.encoderTolerance;
        par.minStartPower = parameters.minStartPower;
        par.minTurnPower  = parameters.minTurnPower;
        par.debug         = parameters.debug;
        circumference     = Math.PI * par.wheelDiameter;
        driveRatio        = par.motorRatio * par.gearRatio;
        crabRatio         = driveRatio * (par.mecanumAngle / 100);


        //Verify linearOpMode first to prevent crashes
        if (parameters.opMode == null) {
            dConfigured = false;
            return (dConfigured);
        }
        par.opMode = parameters.opMode;
        par.opMode.telemetry.addData("Status", "Configuring Drive....");

        //  Verify valid drive type
        if ((parameters.driveType != DriveType.TANK) && (parameters.driveType != DriveType.MECANUM)) {
            par.opMode.telemetry.addData("Invalid Drive Type", parameters.driveType);
            dConfigured = false;
        }

        // Verify valid motors for mecanum
        if ((parameters.driveType == DriveType.MECANUM) &&
                ((parameters.frontRight == null) ||
                 (parameters.frontLeft == null)  ||
                 (parameters.rearRight == null)  ||
                 (parameters.rearLeft == null))) {
            par.opMode.telemetry.addData("Invalid Motor(s) Front Right ", (parameters.frontRight == null));
            par.opMode.telemetry.addData("Invalid Motor(s) Front Left  ", (parameters.frontLeft == null));
            par.opMode.telemetry.addData("Invalid Motor(s) Rear Right  ", (parameters.rearRight == null));
            par.opMode.telemetry.addData("Invalid Motor(s) Rear Left   ", (parameters.rearLeft == null));
            dConfigured = false;
        }

        // Verify valid motors for tank
        if ((parameters.driveType == DriveType.TANK) &&
                 ((parameters.frontRight == null) ||
                  (parameters.frontLeft == null))){
            par.opMode.telemetry.addData("Invalid Motor(s) Front Right ", (parameters.frontRight == null));
            par.opMode.telemetry.addData("Invalid Motor(s) Front Left  ", (parameters.frontLeft == null));
            dConfigured = false;
        }

        // Verify valid imu available.
        if ((parameters.imu == null) || (!parameters.imu.isGyroCalibrated())) {
            par.opMode.telemetry.addData("Invalid IMU              ", (parameters.imu == null));
            par.opMode.telemetry.addData("Invalid IMU Configured   ", parameters.imu.isGyroCalibrated());
            dConfigured = false;
        }

        //Verify marginally sane numbers for motor, wheel and gear ratio
        if ((parameters.gearRatio <= 0) || (parameters.wheelDiameter <=0) || (parameters.motorRatio <=0)) {
            par.opMode.telemetry.addData("Invalid gear ratio     ", parameters.gearRatio);
            par.opMode.telemetry.addData("Invalid motor ratio    ", parameters.motorRatio);
            par.opMode.telemetry.addData("Invalid wheel diameter ", parameters.wheelDiameter);
            dConfigured = false;
        }
        if (parameters.encoderTolerance < parameters.motorRatio/2) {
            par.opMode.telemetry.addData("WARNING -- encoderTolerance too small", parameters.encoderTolerance);
            par.opMode.telemetry.addData("           encoderTolerance set to   ", (int)parameters.motorRatio/2);
            par.encoderTolerance = (int)parameters.motorRatio/2;
        }
        if ((parameters.driveType == DriveType.MECANUM) && (parameters.mecanumAngle <=0) && (parameters.mecanumAngle > 90)) {
            par.opMode.telemetry.addData("Invalid mecanum angle    ", parameters.mecanumAngle);
            dConfigured = false;
        }

        //Check for reasonable minimum power settings
        if (par.minStartPower < 0.3) {
            par.opMode.telemetry.addData("Warning minimum start power may be low", par.minStartPower);
        }
        if (par.minTurnPower < 0.3) {
            par.opMode.telemetry.addData("Warning minimum turn power may be low", par.minTurnPower);
        }

        //Initialize maneuver to complete
        maneuver.status = MoveStatus.AVAILABLE;
        if (dConfigured) {
            par.opMode.telemetry.addLine("     :SUCCESS");
            par.opMode.telemetry.update();
        }
        else {
            par.opMode.telemetry.addLine("FAILED");
            par.opMode.telemetry.update();
        }

        return (dConfigured);
    }


    //Calculate the decoder counts for te requested maneuver
    private int encoderCounts (double distance, MoveType mType) {
        double ratio = driveRatio;
        if ((mType == MoveType.CRABLEFT) || (mType == MoveType.CRABRIGHT)) {
            ratio = crabRatio;
        }
        return (int)(Math.abs(distance) / circumference * ratio);
    }


    //Configure the motors to run with or without the encoders depending on the maneuver.
    //If the maneuver requires the encoder set the encoder.
    //Always configure all of the motors.   This will allow for complex movements to be implemented
    //in the future.
    private void configureMotors () {
        //Set motors to run with encoders at initial power if FORWARD, REVERSE, CRAB
        //Set motors to run without encoders if PIVOT

        par.frontRight.setDirection(par.frPolarity);
        par.frontLeft.setDirection(par.flPolarity);
        par.rearRight.setDirection(par.rrPolarity);
        par.rearLeft.setDirection(par.rlPolarity);

        if ((maneuver.type == MoveType.PIVOTLEFT) || (maneuver.type == MoveType.PIVOTRIGHT)) {
            par.frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            par.frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            if (par.driveType == DriveType.MECANUM) {
                par.rearRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                par.rearLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
        }
        else {
            par.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            par.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            par.frontRight.setTargetPosition(maneuver.rightTarget);
            par.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            par.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            par.frontLeft.setTargetPosition(maneuver.leftTarget);


            if (par.driveType == DriveType.MECANUM) {
                par.rearRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                par.rearLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                par.rearRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                par.rearLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                if (maneuver.type == MoveType.CRABLEFT) {
                    par.frontRight.setTargetPosition(-1* maneuver.rightTarget);
                    par.frontLeft.setTargetPosition(maneuver.leftTarget);
                    par.rearRight.setTargetPosition(maneuver.rearRightTarget);
                    par.rearLeft.setTargetPosition(-1* maneuver.rearLeftTarget);
                }
                else if (maneuver.type == MoveType.CRABRIGHT) {
                    par.frontRight.setTargetPosition(maneuver.rightTarget);
                    par.frontLeft.setTargetPosition(-1* maneuver.leftTarget);
                    par.rearRight.setTargetPosition(-1* maneuver.rearRightTarget);
                    par.rearLeft.setTargetPosition(maneuver.rearLeftTarget);
                }
                else {  // FORWARD, REVERSE
                    par.frontRight.setTargetPosition(maneuver.rightTarget);
                    par.frontLeft.setTargetPosition(maneuver.leftTarget);
                    par.rearRight.setTargetPosition(maneuver.rearRightTarget);
                    par.rearLeft.setTargetPosition(maneuver.rearLeftTarget);
                }
            }

        }
    }


    // Power the two(four) motors with the specified powers.   Each gets its own power to allow
    // for complicated movements in the future.
    private void powerMotors (double rf, double lf, double rr, double lr) {
        par.frontRight.setPower(rf);
        par.frontLeft.setPower(lf);
        if (par.driveType == DriveType.MECANUM) {
            par.rearRight.setPower(rr);
            par.rearLeft.setPower(lr);
        }
        maneuver.rfPower = rf;
        maneuver.lfPower = lf;
        maneuver.rrPower = rr;
        maneuver.lrPower = lr;
    }


    //Navigate between 0 and 360
    public float mod360 (float angle) {
        return ((int)angle - ((int)angle/360)*360);  //Integer math
    }

    //Convert the imu heading data to 0 to 360
    public float getHeading () {
        float temp = par.imu.getAngularOrientation().firstAngle;
        if (temp < 0) {
            temp = Math.abs(temp);
        }
        else if (temp > 0) {
            temp = 360 - temp;
        }
        else {
            temp = 0;  //redundant
        }
        return temp;
    }


    // Four different cases.   Pivoting either left or right and the does the robot have to cross
    // the zero degree threshold.
    private float turnRemaining () {
        float remaining = maneuver.angleTarget - getHeading();

        if ((maneuver.type == MoveType.PIVOTRIGHT) && (remaining < 0)) {
            remaining += 360;  //OK
        }
        else if ((maneuver.type == MoveType.PIVOTLEFT) && (remaining < 0)) {
            remaining = remaining * -1;
        }
        else if ((maneuver.type == MoveType.PIVOTRIGHT) && (remaining > 0)) {
           //Nothing to do, value should be correct.
        }
        else if ((maneuver.type == MoveType.PIVOTLEFT) && (remaining > 0)) {
            remaining = 360 - remaining;  //OK
        }
        else {
            remaining = 0;
        }

        return remaining;
    }







    //Calculates power for drive motors based on the input parameters.  NOTE: does not
    //set power to the motors this is only done in the update section.  Val is
    //interpreted based on the type of movement specified.  Calling move before the
    // previous move is complete will result in the value being overwritten.
    //  FORWARD, CRAB, REVERSE -- val is distance, only positive is accepted.
    //  PIVOT(s) -- val is in degrees and relative to current heading   For example if the current
    //            heading is 100 degrees and val was set to 40 a turn would be made to either 140
    //            or 60 depending on the type of PIVOT requested.   Zero boundary protection is
    //            implemented.  OVERTURN protection IS NOT implemented.   Care should be taken on
    //            pivots with ZERO_DEGREE tolerance.    These may not be possible on certain robot
    //            drivetrains due to a combination of slip, momentum, and braking traction.  Only
    //            the magnitude of val is considered.
    //  startPower is the initial power supplied to the motors.
    public void move (MoveType mType, double val, double startPower)
    {
        double power = startPower;
        double direction = 1;

        //Sanity check the inputs
        if (mType == MoveType.REVERSE) {
            direction = -1;
        }
        if (startPower < par.minStartPower) {
            power = par.minStartPower;
        }
        maneuver.type = mType;

        //Calculate encoder counts
        switch (mType) {
            case CRABLEFT:
            case CRABRIGHT:
            {
                maneuver.rightTarget = encoderCounts(val*direction, mType);
                if (mType == MoveType.CRABLEFT) {
                    maneuver.rightTarget = maneuver.rightTarget * -1;
                }
                maneuver.leftTarget  = -1 * maneuver.rightTarget;
                maneuver.rearRightTarget = -1 * maneuver.rightTarget;
                maneuver.rearLeftTarget = maneuver.rightTarget;
                maneuver.angleTarget = 0;
                maneuver.initialPower = power;
                maneuver.status = MoveStatus.INITIATED;
                break;
            }
            case FORWARD:
            case REVERSE:
            {

                maneuver.rightTarget = encoderCounts(val*direction, mType);
                if (mType == MoveType.REVERSE) {
                    maneuver.rightTarget = -1 * maneuver.rightTarget;
                }
                maneuver.leftTarget  = maneuver.rightTarget;
                maneuver.rearRightTarget = maneuver.rightTarget;
                maneuver.rearLeftTarget = maneuver.rightTarget;
                maneuver.angleTarget = 0;
                maneuver.initialPower = power;
                maneuver.status = MoveStatus.INITIATED;
                break;
            }
            case LEFTFORWARD:
            case LEFTREVERSE:
            {
                maneuver.leftTarget = encoderCounts(val*direction, mType);
                if (mType == MoveType.LEFTREVERSE) {
                    maneuver.leftTarget = -1 * maneuver.leftTarget;
                }
                maneuver.rightTarget  = 0;
                maneuver.rearRightTarget = 0;
                maneuver.rearLeftTarget = maneuver.leftTarget;
                maneuver.angleTarget = 0;
                maneuver.initialPower = power;
                maneuver.status = MoveStatus.INITIATED;
                break;
            }
            case RIGHTFORWARD:
            case RIGHTREVERSE:
            {
                maneuver.rightTarget = encoderCounts(val*direction, mType);
                if (mType == MoveType.RIGHTREVERSE) {
                    maneuver.rightTarget = -1 * maneuver.rightTarget;
                }
                maneuver.leftTarget  = 0;
                maneuver.rearRightTarget = maneuver.rightTarget;
                maneuver.rearLeftTarget = 0;
                maneuver.angleTarget = 0;
                maneuver.initialPower = power;
                maneuver.status = MoveStatus.INITIATED;
                break;
            }
            case PIVOTLEFT:
            case PIVOTRIGHT:
            {
                maneuver.status = MoveStatus.INITIATED;
                maneuver.rightTarget = 0;
                maneuver.leftTarget  = 0;
                maneuver.initialPower = power;
                maneuver.angleTarget = Math.abs(mod360((int)val));
                break;
            }
            case STOP:
            default:
            {
                maneuver.status = MoveStatus.AVAILABLE;
                break;
            }

        }

        if (par.debug) {
            par.opMode.telemetry.addData("Manuever Type", maneuver.type);
            par.opMode.telemetry.addData("Manuever Status", maneuver.status);
            par.opMode.telemetry.update();
        }

    }

    // Returns true if the requested move is complete, false otherwise
    public MoveStatus getMoveStatus ()
    {
        return (maneuver.status);
    }


    // Used for pivots.  Returns the sign to use on motor power settings.  This is because turns
    // do not use encoders.
    double rightSign() {
        if (maneuver.type == MoveType.PIVOTLEFT) {
            return (1);
        }
        return (-1);
    }
    double leftSign() {
        if (maneuver.type == MoveType.PIVOTLEFT) {
            return (-1);
        }
        return (1);
    }

    // Sets power to motors and monitors in the requested move has been completed.
    public void update ()
    {


        // Attempt nothing if not configured
        if (dConfigured) {
            switch (maneuver.status) {
                case INITIATED:
                {
                    configureMotors();
                    if ((maneuver.type == MoveType.PIVOTLEFT) || (maneuver.type == MoveType.PIVOTRIGHT)) {
                        //Do not apply too much power if we are already within backoff window
                        if (turnRemaining() < (par.backoffMultiplier * par.pivotTolerance.ordinal())) {
                            powerMotors(rightSign() * par.minTurnPower,
                                        leftSign()  * par.minTurnPower,
                                        rightSign() * par.minTurnPower,
                                        leftSign()  * par.minTurnPower);
                            backoffStarted = true;
                        }
                        else {
                            powerMotors(rightSign() * maneuver.initialPower,
                                        leftSign()  * maneuver.initialPower,
                                        rightSign() * maneuver.initialPower,
                                        leftSign()  * maneuver.initialPower);
                            backoffStarted = false;
                        }

                    }
                    else if (maneuver.type == MoveType.STOP) {
                        powerMotors(0,0,0,0);
                    }
                    else {
                        powerMotors(maneuver.initialPower, maneuver.initialPower, maneuver.initialPower, maneuver.initialPower);
                    }
                    maneuver.status = MoveStatus.INPROGRESS;
                    break;
                }
                case INPROGRESS:
                {
                    //If anything but pivot wait until the motors stop, for pivot use the
                    // gyro to control motors
                    if ((maneuver.type == MoveType.PIVOTLEFT) || (maneuver.type == MoveType.PIVOTRIGHT)) {
                        float  togo;
                        togo = turnRemaining();

                        if (togo < par.pivotTolerance.ordinal()) {
                            //Stop
                            powerMotors(0,0,0,0);
                            maneuver.status = MoveStatus.COMPLETE;
                        }
                        else if (togo < (par.backoffMultiplier * par.pivotTolerance.ordinal())) {
                            //Start slowing do not slow past minimum speed.
                            powerMotors((Math.abs(maneuver.rfPower * par.turnBackoff) < par.minTurnPower) ? par.minTurnPower * rightSign() : maneuver.rfPower * par.turnBackoff,
                                        (Math.abs(maneuver.lfPower * par.turnBackoff) < par.minTurnPower) ? par.minTurnPower * leftSign()  : maneuver.lfPower * par.turnBackoff,
                                        (Math.abs(maneuver.rrPower * par.turnBackoff) < par.minTurnPower) ? par.minTurnPower * rightSign() : maneuver.rrPower * par.turnBackoff,
                                        (Math.abs(maneuver.lrPower * par.turnBackoff) < par.minTurnPower) ? par.minTurnPower * leftSign()  : maneuver.lrPower * par.turnBackoff);
                            backoffStarted = true;
                        }
                        else {
                            //If the backoff has started and we are in here then we went to far.
                            //Stop the robot and try to make a correcting turn.
                            if (backoffStarted) {
                                powerMotors(0,0,0,0);
                                if (maneuver.type == MoveType.PIVOTLEFT) {
                                    maneuver.type = MoveType.PIVOTRIGHT;
                                }
                                else {
                                    maneuver.type = MoveType.PIVOTLEFT;
                                }
                                maneuver.status = MoveStatus.INITIATED;
                                maneuver.initialPower = par.minTurnPower;
                            }
                        }
                    }
                    else if (maneuver.type == MoveType.STOP) {
                        powerMotors(0,0,0,0);
                        maneuver.status = MoveStatus.COMPLETE;
                    }
                    else {
                        // Encoder based movement, check to see if  the encoders are close enough
                        // to complete the action.   NOTE:  This is required due to the weight and
                        // inertia of the robots.   They rarely will perfectly hit the specified
                        // target position.
                        if (par.driveType == DriveType.TANK) {
                            if ((Math.abs(maneuver.rightTarget  - par.frontRight.getCurrentPosition()) < par.encoderTolerance)  &&
                                (Math.abs(maneuver.leftTarget  - par.frontLeft.getCurrentPosition()) < par.encoderTolerance)) {
                                powerMotors(0,0,0,0);
                                maneuver.status = MoveStatus.COMPLETE;
                            }
                        }
                        else {
                            if ((Math.abs(maneuver.rightTarget  - par.frontRight.getCurrentPosition()) < par.encoderTolerance)  &&
                                (Math.abs(maneuver.leftTarget  - par.frontLeft.getCurrentPosition()) < par.encoderTolerance)    &&
                                (Math.abs(maneuver.rearRightTarget  - par.rearRight.getCurrentPosition()) < par.encoderTolerance) &&
                                (Math.abs(maneuver.rearLeftTarget  - par.rearLeft.getCurrentPosition()) < par.encoderTolerance)) {
                                powerMotors(0,0,0,0);
                                maneuver.status = MoveStatus.COMPLETE;
                            }
                        }
                    }
                    break;
                }
                case COMPLETE:
                default:
                    powerMotors (0.0, 0.0, 0.0, 0.0);
                    break;

            }

            if (par.debug == true) {
                par.opMode.telemetry.clearAll();
                par.opMode.telemetry.addData("Current Heading    ", getHeading());
                par.opMode.telemetry.addData("Target Heading     ", maneuver.angleTarget);
                par.opMode.telemetry.addData("Angle Remaining    ", turnRemaining());
                par.opMode.telemetry.addData("Angle Tolerance    ", par.pivotTolerance.ordinal());
                par.opMode.telemetry.addData("Rigt Front Ecnoder ", par.frontRight.getCurrentPosition());
                par.opMode.telemetry.addData("Right Front Target ", maneuver.rightTarget);
                par.opMode.telemetry.addData("Right Front Motor  ", maneuver.rfPower);
                //par.opMode.telemetry.addData("Left Front Motor   ", maneuver.lfPower);
                //par.opMode.telemetry.addData("Right Rear Motor   ", maneuver.rrPower);
                //par.opMode.telemetry.addData("Left Rear Motor    ", maneuver.lrPower);
            }
        }


    }


}
