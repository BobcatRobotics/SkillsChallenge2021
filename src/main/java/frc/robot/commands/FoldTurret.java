package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Turret;

public class FoldTurret extends CommandBase {

    public Turret turret;
    private final XboxController gamepad;

    public FoldTurret(Turret turret, XboxController gpd) {
        this.turret = turret;
        this.gamepad = gpd;
        addRequirements(turret);
    }

    @Override
    public void execute() {
        //If not folded already and user is pressing A button
        if (!turret.getFolded() && gamepad.getRawButtonPressed(Constants.A_Button)) {
            turret.foldTurret();
        }
    }
}