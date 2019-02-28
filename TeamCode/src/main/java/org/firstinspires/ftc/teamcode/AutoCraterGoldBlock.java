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

            new AutoCommand(AutonomousStates.AutoStates.DETECTGOLD, Drive.MoveType.STOP, 0,0,0,0, 5000),
            new AutoCommand(AutonomousStates.AutoStates.LOWER, Drive.MoveType.STOP, 0.9,0,0,0, 10000),
            new AutoCommand(AutonomousStates.AutoStates.MOVE, Drive.MoveType.PIVOTRIGHT, 45, 0.8,0,0,1000),
            new AutoCommand(AutonomousStates.AutoStates.MOVE, Drive.MoveType.FORWARD, 4, 0.8, 0,0, 1000),
            new AutoCommand(AutonomousStates.AutoStates.MOVE, Drive.MoveType.PIVOTLEFT, 0, 0.8,0,0,2000),

            new AutoCommand(AutonomousStates.AutoStates.MOVE, Drive.MoveType.FORWARD, 20, 0.8, 0,0, 2000),
            new AutoCommand(AutonomousStates.AutoStates.MOVE, Drive.MoveType.REVERSE, 5, 0.8, 0,0, 2000),
            new AutoCommand(AutonomousStates.AutoStates.MOVE, Drive.MoveType.PIVOTLEFT, 270, 0.8,0,0,2000),
            new AutoCommand(AutonomousStates.AutoStates.MOVE, Drive.MoveType.FORWARD, 50, 0.8, 0,0, 5000),
            new AutoCommand(AutonomousStates.AutoStates.MOVE, Drive.MoveType.PIVOTLEFT, 225, 0.8,0,0,2000),
            new AutoCommand(AutonomousStates.AutoStates.MOVE, Drive.MoveType.FORWARD, 25, 0.8, 0,0, 5000),
            new AutoCommand(AutonomousStates.AutoStates.TOKEN, Drive.MoveType.STOP, 0.1,0,0,0,2000),
            new AutoCommand(AutonomousStates.AutoStates.TOKEN, Drive.MoveType.STOP, 1.0,0,0,0,500),
            //new AutoCommand(AutonomousStates.AutoStates.MOVE, Drive.MoveType.PIVOTRIGHT, 45, 0.8,0,0,1000),
            new AutoCommand(AutonomousStates.AutoStates.MOVE, Drive.MoveType.REVERSE, 90, 1.0, 0,0, 3500),






            new AutoCommand( AutonomousStates.AutoStates.WAIT, Drive.MoveType.STOP,         18, 0.5, 0,0, 25000),

    };

    public void runOpMode() {
        runMe.runOpMode(this, hardwareMap, cmd);
    }

}