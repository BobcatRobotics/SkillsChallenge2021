package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.commands.auto.DriveStraight;
import frc.robot.commands.auto.PushBot;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class SimpleAuto extends SequentialCommandGroup {

    public SimpleAuto(Shooter shooter, Drivetrain dt, Double speed, Double shootTime, Double driveTime, Feeder feeder, Intake intake, Boolean push){
        addCommands(
            
            new SpinUpShooter(shooter),

            new ShootShooter(shooter, feeder, intake, shootTime),

            new PushBot(push, dt, -speed, driveTime),

            new DriveStraight(dt,speed,driveTime,push)
        );
    }
    
}