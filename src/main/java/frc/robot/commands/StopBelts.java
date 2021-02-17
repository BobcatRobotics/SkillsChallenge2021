package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class StopBelts extends CommandBase {

    private Intake intake;
    private Feeder feeder;
    private Shooter shooter;
    private final XboxController gamepad;

    public StopBelts(Intake itk, Feeder fdr, Shooter str, XboxController gpd) {
        
        this.intake = itk;
        this.feeder = fdr;
        this.shooter = str;
        this.gamepad = gpd;
        addRequirements(intake, feeder, shooter);
    }

    @Override
    public void execute() {
        //If user is pressing d pad down
        int dpadval = gamepad.getPOV();

        intake.stopLowerTower();
        intake.stopFunnel();
        intake.stopIntake();
        shooter.stopShooter();
        feeder.stop();
        
    }
}