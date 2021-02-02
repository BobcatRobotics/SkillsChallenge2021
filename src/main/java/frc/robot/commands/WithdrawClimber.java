package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;
public class WithdrawClimber extends CommandBase {

    private Climber climber;
    private final Joystick leftstick;

    public WithdrawClimber(Climber cl, Joystick j) {
        
        this.climber = cl;
         
        this.leftstick = j;
        addRequirements(climber);
    }

    @Override
    public void execute() {
        //Not assigned to any button as of right now
        double speed = leftstick.getRawAxis(Joystick.AxisType.kY.value);
        if(speed < 0.0)
            speed = 0.0;
        climber.climb(speed);
    }
}