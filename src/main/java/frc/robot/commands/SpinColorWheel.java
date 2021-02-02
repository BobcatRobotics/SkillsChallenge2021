package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ColorWheel;

public class SpinColorWheel extends CommandBase {

    private ColorWheel colorWheel;
    // bprivate final XboxController gamepad;

    public SpinColorWheel(ColorWheel clw) {
        
        this.colorWheel = clw;
        addRequirements(colorWheel);
    }

    @Override
    public void execute() {
        //Nothing yet
        if (false) {
            colorWheel.spin();
        }
    }

    @Override
    public void end(boolean interrupted) {
        colorWheel.stop();
    }
}