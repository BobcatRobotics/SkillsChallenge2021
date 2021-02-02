package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class PancakeController extends CommandBase {

    public Shooter shooter;
    private final XboxController gamepad;

    public PancakeController(Shooter str, XboxController gpd) {
        this.shooter = str;
        this.gamepad = gpd;
        addRequirements(shooter);
    }

    @Override
    public void execute() {
        int dpadval = gamepad.getPOV();

        if(dpadval >= 45 && dpadval <= 135){
            shooter.setPancake(true);
        } else if(dpadval >= 225 && dpadval <=315){
            shooter.setPancake(true);
        }
    }
}