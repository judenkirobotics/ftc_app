package org.firstinspires.ftc.teamcode;

/*
 *  EXAMPLE AUTONOMOUS FILE
 *
 *
 *  Refer to AutonomousStates.java for the list of valid commands and how to pass parameters.
 *
 */



        import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
        import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
        import com.qualcomm.robotcore.hardware.DcMotor;
        import com.qualcomm.robotcore.hardware.DcMotorSimple;
        import com.qualcomm.robotcore.util.ElapsedTime;


//@Autonomous(name="JK Autonomous Example", group="Pushbot")
@SuppressWarnings("WeakerAccess")
public class JK_AutonomousExample extends LinearOpMode {

    AutonomousStates runMe = new AutonomousStates();
    AutoCommand cmd[] = {
            new AutoCommand( AutonomousStates.AutoStates.MOVE, Drive.MoveType.FORWARD,      18, 0.1, 0, 0,5000),
            new AutoCommand( AutonomousStates.AutoStates.MOVE, Drive.MoveType.REVERSE,      18, 0.7, 0, 0,5000),
            new AutoCommand( AutonomousStates.AutoStates.MOVE, Drive.MoveType.RIGHTFORWARD, 18, 0.8, 0, 0,5000),
            new AutoCommand( AutonomousStates.AutoStates.MOVE, Drive.MoveType.RIGHTREVERSE, 18, 0.8, 0, 0,5000),
            new AutoCommand( AutonomousStates.AutoStates.MOVE, Drive.MoveType.LEFTFORWARD,  18, 0.4, 0, 0,5000),
            new AutoCommand( AutonomousStates.AutoStates.MOVE, Drive.MoveType.LEFTREVERSE,  18, 0.4, 0, 0,5000),
            new AutoCommand( AutonomousStates.AutoStates.MOVE, Drive.MoveType.PIVOTLEFT,    270,1.0, 0, 0,5000),
            new AutoCommand( AutonomousStates.AutoStates.MOVE, Drive.MoveType.PIVOTRIGHT,   0,  1.0, 0, 0,5000),
            new AutoCommand( AutonomousStates.AutoStates.MOVE, Drive.MoveType.FORWARD,      2,  0.5, 0, 0,5000),
            new AutoCommand( AutonomousStates.AutoStates.MOVE, Drive.MoveType.REVERSE,      2,  0.5, 0, 0,5000),
            new AutoCommand( AutonomousStates.AutoStates.WAIT, Drive.MoveType.STOP,         18, 0.5, 0, 0,5000),
    };

    public void runOpMode() {
        runMe.runOpMode(this, hardwareMap, cmd);
    }

}