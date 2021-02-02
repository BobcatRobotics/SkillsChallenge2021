package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;

public class IntakeOut extends CommandBase {

    private Intake intake;
    private final Joystick gamepad;

    /**
     * 1. Changes the intake direction to take in balls from the ground. <br>
     * 2. Hold the right button to activate the command. <br>
     * 3. Balls can be pushed out onto the ground. <br>
     * 4. This changes the intake direction, not whether it is stowed out or not.
     */
    public IntakeOut(Intake itk, Joystick gpd) {
        this.intake = itk;
        this.gamepad = gpd;
        addRequirements(intake);
    }

    @Override
    public void execute() {
        //If right trigger pressed and NOT right bumper
        if (gamepad.getRawButton(Constants.Right_Trigger_Button) &&
                 !gamepad.getRawButton(Constants.Right_Bumper_Button)){
            intake.runIntakeOut(true);
        }
    }
}