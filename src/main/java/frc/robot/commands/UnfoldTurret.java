package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Turret;

public class UnfoldTurret extends CommandBase {

    public Turret turret;
    private final XboxController gamepad;

    public UnfoldTurret(Turret turret, XboxController gpd) {
        this.turret = turret;
        this.gamepad = gpd;
        addRequirements(turret);
    }

    @Override
    public void execute() {
        //If folded and user is pressing B button
        if (turret.getFolded() && gamepad.getRawButtonPressed(Constants.B_Button)) {
            turret.unfoldTurret();
        }
    }
}