package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;
public class WithdrawClimber extends CommandBase {

    private Climber climber;
    private final XboxController gamepad;

    public WithdrawClimber(Climber cl, XboxController gpd) {
        
        this.climber = cl;
         
        this.gamepad= gpd;
        addRequirements(climber);
    }

    @Override
    public void execute() {
        //Not assigned to any button as of right now
        double speed = gamepad.getY(Hand.kLeft);
        if(speed < 0.0)
            speed = 0.0;
        climber.climb(speed);
    }
}