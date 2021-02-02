package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

public class PullClimber extends CommandBase {

    private Climber climber;
    private final XboxController gamepad;

    public PullClimber(Climber cl, XboxController gpd) {

        this.climber = cl;
        this.gamepad = gpd;
        addRequirements(climber);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        //If the climber is deployed AND left joystick is pressed AND left joystick is aimed down
        // if (gamepad.getRawButton(Constants.Left_Joystick_Pressed) && climber.getDeployed() && (gamepad.getRawAxis(Constants.Left_Joystick_Y_Axis) < 0)) {
        //     climber.climb();
        // }
    }

    @Override
    public void end(boolean interrupted) {
        if(interrupted) {
            climber.stop();
        }
    }
}