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

//@Autonomous(name="Zone Dozer Color", group="Pushbot")
@SuppressWarnings("WeakerAccess")
public class AutoZoneDozerColor extends LinearOpMode {

    AutonomousStates runMe = new AutonomousStates();
    AutoCommand cmd[] =    {
            // detect color
            new AutoCommand(AutonomousStates.AutoStates.DETECTGOLD, Drive.MoveType.STOP, 0, 0, 0, 0, 5000),
            // descent
            new AutoCommand(AutonomousStates.AutoStates.LOWER, Drive.MoveType.STOP, 0.9,0,0,0, 10000),

            // clear the hook
            new AutoCommand(AutonomousStates.AutoStates.MOVE, Drive.MoveType.PIVOTRIGHT, 45, 0.8,0,0,1000),
            new AutoCommand(AutonomousStates.AutoStates.MOVE, Drive.MoveType.FORWARD, 4, 0.8, 0,0, 1000),
            new AutoCommand(AutonomousStates.AutoStates.MOVE, Drive.MoveType.PIVOTLEFT, 0, 0.8,0,0,1000),

            //procede to the depot corner
            new AutoCommand(AutonomousStates.AutoStates.MOVE, Drive.MoveType.PIVOTRIGHT, 5, 0.8,0,0,1000),
            new AutoCommand(AutonomousStates.AutoStates.MOVE, Drive.MoveType.FORWARD, 30, 0.8, 0,0, 2000),
            new AutoCommand(AutonomousStates.AutoStates.MOVE, Drive.MoveType.PIVOTLEFT, 355, 0.8,0,0,1000),
            new AutoCommand(AutonomousStates.AutoStates.MOVE, Drive.MoveType.FORWARD, 30, 0.8, 0,0, 2000),

            // prepare to dump token
            new AutoCommand(AutonomousStates.AutoStates.MOVE, Drive.MoveType.PIVOTRIGHT, 45, 0.8,0,0,1000),

            // dump token and recover the dumper
            new AutoCommand(AutonomousStates.AutoStates.TOKEN, Drive.MoveType.STOP, 0.1,0,0,0, 2000),
            new AutoCommand(AutonomousStates.AutoStates.TOKEN, Drive.MoveType.STOP, 1.0,0,0,0,500),

            // go to the crater
            new AutoCommand(AutonomousStates.AutoStates.MOVE, Drive.MoveType.REVERSE, 80, 1.0, 0,0, 7500),
            new AutoCommand( AutonomousStates.AutoStates.WAIT, Drive.MoveType.STOP,         18, 0.5, 0,0, 25000),
    };

    public void runOpMode() {
        runMe.runOpMode(this, hardwareMap, cmd);
    }
}