package org.firstinspires.ftc.teamcode;

/*
 * Created by
 * James K Rumsey
 * December 2018
 *
 * Purpose: Absurdity for absurdities sake has value.
 *
 * You put your right treads in
 * You put your right treads out
 * You put your right treads in and you shake it all about
 * You do the Hokey Pokey and you turn yourself around
 *     that's what its all about.  (CLAP)
 *
 * You put your left treads in
 * You put your left treads out
 * You put your left treads in and you shake it all about
 * You do the Hokey Pokey and you turn yourself around
 *     that's what its all about.    (CLAP)
 *
 * You put your robot in
 * You put your robot out
 * You put your robot in and you shake it all about
 * You do the Hokey Pokey and you turn yourself around
 *     that's what its all about.    (CLAP)
 *
 */




import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


//@Autonomous(name="Zone Dozer", group="Pushbot")
@SuppressWarnings("WeakerAccess")
public class AutoZoneDozer extends LinearOpMode {

    AutonomousStates runMe = new AutonomousStates();
    AutoCommand cmd[] = {

            new AutoCommand(AutonomousStates.AutoStates.LOWER, Drive.MoveType.STOP, 0.9,0,0,0, 10000),
            new AutoCommand(AutonomousStates.AutoStates.MOVE, Drive.MoveType.PIVOTRIGHT, 45, 0.8,0,0,1000),
            new AutoCommand(AutonomousStates.AutoStates.MOVE, Drive.MoveType.FORWARD, 4, 0.8, 0,0, 1000),
            new AutoCommand(AutonomousStates.AutoStates.MOVE, Drive.MoveType.PIVOTLEFT, 0, 0.8,0,0,1000),
            new AutoCommand(AutonomousStates.AutoStates.MOVE, Drive.MoveType.FORWARD, 60, 0.8, 0,0, 4000),
            //new AutoCommand(AutonomousStates.AutoStates.TOKEN, Drive.MoveType.STOP, 0.4,0,0,0,500),
            new AutoCommand(AutonomousStates.AutoStates.MOVE, Drive.MoveType.PIVOTRIGHT, 45, 0.8,0,0,1000),
            new AutoCommand(AutonomousStates.AutoStates.TOKEN, Drive.MoveType.STOP, 0.1,0,0,0, 2000),
            new AutoCommand(AutonomousStates.AutoStates.TOKEN, Drive.MoveType.STOP, 1.0,0,0,0,500),
            new AutoCommand(AutonomousStates.AutoStates.MOVE, Drive.MoveType.REVERSE, 80, 1.0, 0,0, 7500),

            //new AutoCommand(AutonomousStates.AutoStates.DETECT, Drive.MoveType.STOP, 0,0,0,0, 500),
            //new AutoCommand(AutonomousStates.AutoStates.PADDLE, Drive.MoveType.STOP, 1.0,0,0,0, 1000),
            //new AutoCommand(AutonomousStates.AutoStates.SWING, Drive.MoveType.STOP, 0.6,0,0,0,500),
            //new AutoCommand(AutonomousStates.AutoStates.DETECT, Drive.MoveType.STOP, 0,0,0,0, 500),
            //new AutoCommand(AutonomousStates.AutoStates.PADDLE, Drive.MoveType.STOP, 1.0,0,0,0, 1000),
            /*new AutoCommand(AutonomousStates.AutoStates.MOVE, Drive.MoveType.PIVOTLEFT, 270, 0.7,0,0, 2000),
             new AutoCommand(AutonomousStates.AutoStates.MOVE, Drive.MoveType.FORWARD, 45, 0.8, 0,0, 3500),
             new AutoCommand(AutonomousStates.AutoStates.MOVE, Drive.MoveType.PIVOTLEFT, 230, 0.7,0,0, 2000),
             new AutoCommand(AutonomousStates.AutoStates.MOVE, Drive.MoveType.FORWARD, 40, 0.8, 0,0, 10000),
             //new AutoCommand(AutonomousStates.AutoStates.MOVE, Drive.MoveType.PIVOTRIGHT, 112.5, 0.7,0,0, 1000),
             new AutoCommand(AutonomousStates.AutoStates.TOKEN, Drive.MoveType.STOP, 0.1, 0, 0,0,1000),
             new AutoCommand(AutonomousStates.AutoStates.TOKEN, Drive.MoveType.STOP, 1.0, 0, 0,0,50),
             new AutoCommand(AutonomousStates.AutoStates.MOVE, Drive.MoveType.PIVOTLEFT, 220, 0.7,0,0, 2000),
             new AutoCommand(AutonomousStates.AutoStates.MOVE, Drive.MoveType.REVERSE, 40, 1.0, 0,0, 3500),
             new AutoCommand(AutonomousStates.AutoStates.MOVE, Drive.MoveType.PIVOTLEFT, 220, 0.7,0,0, 2000),
             new AutoCommand(AutonomousStates.AutoStates.MOVE, Drive.MoveType.REVERSE, 90, 1.0, 0,0, 3500),
            //original comment
             //Right in, out, in
             new AutoCommand( AutonomousStates.AutoStates.MOVE, Drive.MoveType.RIGHTFORWARD, 18, 0.9, 0,0, 2000),
             new AutoCommand( AutonomousStates.AutoStates.MOVE, Drive.MoveType.RIGHTREVERSE, 18, 0.9, 0,0, 2000),
             new AutoCommand( AutonomousStates.AutoStates.MOVE, Drive.MoveType.RIGHTFORWARD, 18, 0.9, 0,0, 2000),
             //Color Arm
             new AutoCommand( AutonomousStates.AutoStates.SWING, Drive.MoveType.STOP,       0.7, 0,  0,0, 500),
             new AutoCommand( AutonomousStates.AutoStates.SWING, Drive.MoveType.STOP,       0.6, 0,  0,0, 120),
             new AutoCommand( AutonomousStates.AutoStates.SWING, Drive.MoveType.STOP,       0.7, 0,  0,0, 120),
             new AutoCommand( AutonomousStates.AutoStates.SWING, Drive.MoveType.STOP,       0.6, 0,  0,0, 120),
             new AutoCommand( AutonomousStates.AutoStates.SWING, Drive.MoveType.STOP,       0.7, 0,  0,0, 120),
             new AutoCommand( AutonomousStates.AutoStates.SWING, Drive.MoveType.STOP,       0.6, 0,  0,0, 120),
             new AutoCommand( AutonomousStates.AutoStates.SWING, Drive.MoveType.STOP,       0.7, 0,  0,0, 120),
             new AutoCommand( AutonomousStates.AutoStates.SWING, Drive.MoveType.STOP,       0.0, 0,  0,0, 40),
             //Turn around
             new AutoCommand( AutonomousStates.AutoStates.MOVE, Drive.MoveType.PIVOTLEFT,    0,0.5, 0,0, 5000),
             new AutoCommand( AutonomousStates.AutoStates.MOVE, Drive.MoveType.STOP, 18, 0.9, 0,0, 500),



             //Left in, out, in
             new AutoCommand( AutonomousStates.AutoStates.MOVE, Drive.MoveType.LEFTFORWARD,  18, 0.9, 0,0, 2000),
             new AutoCommand( AutonomousStates.AutoStates.MOVE, Drive.MoveType.LEFTREVERSE,  18, 0.9, 0,0, 2000),
             new AutoCommand( AutonomousStates.AutoStates.MOVE, Drive.MoveType.LEFTFORWARD,  18, 0.9, 0,0, 2000),
             //Color Arm
             new AutoCommand( AutonomousStates.AutoStates.SWING, Drive.MoveType.STOP,       0.7, 0,  0,0, 500),
             new AutoCommand( AutonomousStates.AutoStates.SWING, Drive.MoveType.STOP,       0.6, 0,  0,0, 120),
             new AutoCommand( AutonomousStates.AutoStates.SWING, Drive.MoveType.STOP,       0.7, 0,  0,0, 120),
             new AutoCommand( AutonomousStates.AutoStates.SWING, Drive.MoveType.STOP,       0.6, 0,  0,0, 120),
             new AutoCommand( AutonomousStates.AutoStates.SWING, Drive.MoveType.STOP,       0.7, 0,  0,0, 120),
             new AutoCommand( AutonomousStates.AutoStates.SWING, Drive.MoveType.STOP,       0.6, 0,  0,0, 120),
             new AutoCommand( AutonomousStates.AutoStates.SWING, Drive.MoveType.STOP,       0.7, 0,  0,0, 120),
             new AutoCommand( AutonomousStates.AutoStates.SWING, Drive.MoveType.STOP,       0.0, 0,  0,0, 40),
             //Turn around
             new AutoCommand( AutonomousStates.AutoStates.MOVE, Drive.MoveType.PIVOTRIGHT,   0,  0.5, 0,0, 5000),
             new AutoCommand( AutonomousStates.AutoStates.MOVE, Drive.MoveType.STOP, 18, 0.9, 0,0, 500),


             //Whole in, out, in
             new AutoCommand( AutonomousStates.AutoStates.MOVE, Drive.MoveType.FORWARD,      18,  0.9, 0,0, 2000),
             new AutoCommand( AutonomousStates.AutoStates.MOVE, Drive.MoveType.REVERSE,      18,  0.9, 0,0, 2000),
             new AutoCommand( AutonomousStates.AutoStates.MOVE, Drive.MoveType.FORWARD,      18,  0.9, 0,0, 2000),
             //Color Arm
             new AutoCommand( AutonomousStates.AutoStates.SWING, Drive.MoveType.STOP,       0.7, 0,  0,0, 500),
             new AutoCommand( AutonomousStates.AutoStates.SWING, Drive.MoveType.STOP,       0.6, 0,  0,0, 120),
             new AutoCommand( AutonomousStates.AutoStates.SWING, Drive.MoveType.STOP,       0.7, 0,  0,0, 120),
             new AutoCommand( AutonomousStates.AutoStates.SWING, Drive.MoveType.STOP,       0.6, 0,  0,0, 120),
             new AutoCommand( AutonomousStates.AutoStates.SWING, Drive.MoveType.STOP,       0.7, 0,  0,0, 120),
             new AutoCommand( AutonomousStates.AutoStates.SWING, Drive.MoveType.STOP,       0.6, 0,  0,0, 120),
             new AutoCommand( AutonomousStates.AutoStates.SWING, Drive.MoveType.STOP,       0.7, 0,  0,0, 120),
             new AutoCommand( AutonomousStates.AutoStates.SWING, Drive.MoveType.STOP,       0.0, 0,  0,0, 40),
             //Turn around
             new AutoCommand( AutonomousStates.AutoStates.MOVE, Drive.MoveType.PIVOTRIGHT,   355,  0.5, 0,0, 5000),
             new AutoCommand( AutonomousStates.AutoStates.MOVE, Drive.MoveType.PIVOTRIGHT,   0,  0.5, 0,0, 5000),
             */

            new AutoCommand( AutonomousStates.AutoStates.WAIT, Drive.MoveType.STOP,         18, 0.5, 0,0, 25000),

    };

    public void runOpMode() {
        runMe.runOpMode(this, hardwareMap, cmd);
    }

}