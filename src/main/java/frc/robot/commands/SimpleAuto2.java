package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.commands.auto.DriveStraight;
import frc.robot.commands.auto.PushBot;
import frc.robot.subsystems.*;

public class SimpleAuto2 extends SequentialCommandGroup {

    public SimpleAuto2(Shooter shooter, Turret turret, Drivetrain dt, Double speed, Double shootTime, Double driveTime, Feeder feeder, Intake intake, Boolean push, Double waitTime){
        addCommands(
            
            new SpinUpShooter(shooter),

            new Wait(waitTime),

            new SpinUpShooter(shooter),

            new DriveStraight(dt,speed, 2 * driveTime,push),

            new ShootShooter(shooter, feeder, intake, shootTime)
            
        );
    }
    
}