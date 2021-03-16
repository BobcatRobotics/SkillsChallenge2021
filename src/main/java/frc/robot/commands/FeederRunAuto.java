package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class FeederRunAuto extends CommandBase {
    private Feeder feeder;
    private Intake intake;
    private Shooter shooter;
    /**
     * 1. This feeds the balls to the shooter mechanism. <br>
     * 2. This starts the motors on the feeder. <br>
     * 3. The feeder is the system that takes the balls up into the shooter.
     */
    public FeederRunAuto(Feeder fd, Intake itk, Shooter str, Joystick gpd) {
        this.intake = itk;
        this.feeder = fd;
        this.shooter = str;
        addRequirements(intake, feeder, shooter);
    }

    @Override
    public void execute() {
        //Look at the README.md for the rulesw that the below logic should match
        boolean lowerBallPresent = intake.getBallPresent();
        boolean upperBallPresent = feeder.getBallPresent();
        Joystick j = RobotContainer.rightStick;
        //boolean lowerBallPresent = false;
        //boolean upperBallPresent = false;
        //No balls in tower at all
        if (shooter.isRunning() && (shooter.atSpeed())) {
            feeder.run(true);
            intake.runFunnelIn(true);
            intake.runLowerTowerIn(true);
        }else{
            if ((!lowerBallPresent && !upperBallPresent) || (lowerBallPresent && !upperBallPresent)) {
                feeder.run(false);
                intake.runFunnelIn(false);
                intake.runLowerTowerIn(false);
            //Ball at upper tower but not lower
            } else if (!lowerBallPresent && upperBallPresent) {
                intake.runFunnelIn(false);
                intake.runLowerTowerIn(false);
                feeder.stop();
            //We have balls at upper and lower
            } else if (lowerBallPresent && upperBallPresent) {
                feeder.stop();
                intake.stopFunnel();
                intake.stopLowerTower();
            }                   
        }  
    }

    public void feed(){
        execute();
    }
}