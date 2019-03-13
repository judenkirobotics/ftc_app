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


@Autonomous(name="Crater Dozer", group="Pushbot")
@SuppressWarnings("WeakerAccess")
public class AutoCraterGoldBlock extends LinearOpMode {

    AutonomousStates runMe = new AutonomousStates();
    AutoCommand cmd[] = {

            // GET OFF THE HOOK
            new AutoCommand(AutonomousStates.AutoStates.DETECTGOLD, Drive.MoveType.STOP, 0,0,0,0, 1000),
            new AutoCommand(AutonomousStates.AutoStates.LOWER, Drive.MoveType.STOP, 0.9,0,0,0, 10000),
            new AutoCommand(AutonomousStates.AutoStates.MOVE, Drive.MoveType.PIVOTRIGHT, 50, 0.8,0,0,1000),
            new AutoCommand(AutonomousStates.AutoStates.MOVE, Drive.MoveType.FORWARD, 10, 0.8, 0,0, 1000),
            new AutoCommand(AutonomousStates.AutoStates.MOVE, Drive.MoveType.PIVOTLEFT, 20, 0.8,0,0,2000),

            //PUSH THE BLOCK
            new AutoCommand(AutonomousStates.AutoStates.PUSHBLOCK, Drive.MoveType.STOP,0, 0, 0, 0, 1000),
            new AutoCommand(AutonomousStates.AutoStates.BLOCKPIVOT, Drive.MoveType.STOP,0, 0, 0, 0, 2000),
            new AutoCommand(AutonomousStates.AutoStates.BLOCKFORWARD, Drive.MoveType.STOP,0, 0, 0, 0, 2000),
            new AutoCommand(AutonomousStates.AutoStates.BLOCKREVERSE, Drive.MoveType.STOP,0, 0, 0, 0, 2000),

            //new AutoCommand(AutonomousStates.AutoStates.MOVE, Drive.MoveType.FORWARD, 20, 0.8, 0,0, 2000),
            //new AutoCommand(AutonomousStates.AutoStates.MOVE, Drive.MoveType.REVERSE, 5, 0.8, 0,0, 2000),
            new AutoCommand(AutonomousStates.AutoStates.MOVE, Drive.MoveType.PIVOTLEFT, 300, 0.9,0,0,2000),
            new AutoCommand(AutonomousStates.AutoStates.MOVE, Drive.MoveType.FORWARD, 37, 1.0, 0,0, 5000),
            new AutoCommand(AutonomousStates.AutoStates.MOVE, Drive.MoveType.PIVOTLEFT, 235, 0.9,0,0,2000),
            new AutoCommand(AutonomousStates.AutoStates.MOVE, Drive.MoveType.FORWARD, 41, 1.0, 0,0, 5000),
            new AutoCommand(AutonomousStates.AutoStates.TOKEN, Drive.MoveType.STOP, 0.1,0,0,0,800),
            new AutoCommand(AutonomousStates.AutoStates.TOKEN, Drive.MoveType.STOP, 1.0,0,0,0,500),
            new AutoCommand(AutonomousStates.AutoStates.MOVE, Drive.MoveType.PIVOTLEFT, 227, 0.8,0,0,300),
            new AutoCommand(AutonomousStates.AutoStates.MOVE, Drive.MoveType.REVERSE, 90, 1.0, 0,0, 4500),






            new AutoCommand( AutonomousStates.AutoStates.WAIT, Drive.MoveType.STOP,         18, 0.5, 0,0, 25000),

    };

    public void runOpMode() {
        runMe.runOpMode(this, hardwareMap, cmd);
    }

}