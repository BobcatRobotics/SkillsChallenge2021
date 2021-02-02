package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class PancakeDown extends CommandBase {

    public Shooter shooter;
    private final XboxController gamepad;

    public PancakeDown(Shooter str, XboxController gpd) {
        this.shooter = str;
        this.gamepad = gpd;
        addRequirements(shooter);
    }

    @Override
    public void execute() {
        if(!shooter.getPancakeExpanded())
            shooter.setPancake(false);
    }
}