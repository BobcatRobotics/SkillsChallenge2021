package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class PancakeUp extends CommandBase {

    public Shooter shooter;

    public PancakeUp(Shooter str, XboxController gpd) {
        this.shooter = str;
        addRequirements(shooter);
    }

    @Override
    public void execute() {
        if(!shooter.getPancakeExpanded())
            shooter.setPancake(true);
    }
}