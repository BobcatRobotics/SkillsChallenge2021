package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;
public class StopClimber extends CommandBase {

    private Climber climber;

    public StopClimber(Climber cl) {
        
        this.climber = cl;
         
        addRequirements(climber);
    }

    @Override
    public void execute() {
        //Not assigned to any button as of right now
        climber.stop() ;
    }
}