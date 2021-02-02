package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class StopShooter extends CommandBase {
    public Shooter shooter;
    public StopShooter(Shooter str){
        shooter = str;
    }

    @Override
    public void execute(){
        shooter.stopShooter();
    }

    @Override
    public boolean isFinished(){
        return true;
    }


}