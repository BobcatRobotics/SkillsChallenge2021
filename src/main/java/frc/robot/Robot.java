/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.*;
import frc.robot.commands.*;

public class Robot extends TimedRobot {
  
  private DriveTele drivetele = new DriveTele(RobotContainer.drivetrain, RobotContainer.rightStick, RobotContainer.leftStick);
  //private MoveTurret moveTurret = new MoveTurret(RobotContainer.turret, RobotContainer.gamepad);
  private StopAtCollision stopAtCollision = new StopAtCollision(RobotContainer.navx, RobotContainer.drivetrain);
  // Default Limelight modes for when the different robot states initialize
  // 0: Initial State (RobotInit)
  // 1: Disabled
  // 2: Autonomous
  // 3: TeleOp
  // 4: Test
  private final Limelight.LED[] defaultLED = {
    Limelight.LED.OFF,
    Limelight.LED.OFF,
    Limelight.LED.ON,
    Limelight.LED.OFF,
    Limelight.LED.ON
  };
  
  private final Limelight.CAM[] defaultCAM = {
    Limelight.CAM.DRIVER,
    Limelight.CAM.DRIVER,
    Limelight.CAM.VISION,
    Limelight.CAM.DRIVER,
    Limelight.CAM.VISION
  };

  private SendableChooser<String> autoChooser;
  private SendableChooser<String> pushBotChooser;
  private Limelight limelight;
  private RobotContainer m_robotContainer;
  private Joystick gamepad;
  private Joystick rightStick;
  private Intake intake;
  private Shooter shooter;
  private Drivetrain drivetrain;
  private Feeder feeder;
  private Climber climber;
  private Turret turret;
  private ColorWheel colorWheel;
  private CommandBase desiredAutoCommand = null;
  private ShuffleboardTab tab = Shuffleboard.getTab("Things Tab");
  private NetworkTableEntry waitNT = tab.add("WaitTime",0.0).getEntry();
  private NetworkTableEntry sideNT = tab.add("Direction","Right").getEntry();
  private double waitTime = 0;
  private NavxGyro navx;

  @Override
  public void robotInit() {
    // limelight = m_robotContainer.limelight;
    // limelight.setLED(1);
    // limelight.setCAM(0);
    m_robotContainer = new RobotContainer();
    gamepad = m_robotContainer.gamepad;
    rightStick = m_robotContainer.rightStick;
    // intake = m_robotContainer.intake;
    // shooter = m_robotContainer.shooter;
    drivetrain = m_robotContainer.drivetrain;
    // feeder = m_robotContainer.feeder;
    // climber = m_robotContainer.climber;
    // turret = m_robotContainer.turret;
    // colorWheel = m_robotContainer.colorwheel;
    autoChooser = new SendableChooser<String>();
    pushBotChooser = new SendableChooser<String>();
    navx = m_robotContainer.navx;

    // autoChooser.setDefaultOption("Shoot & Scoot", Constants.SHOOT_SCOOT);
    // autoChooser.setName("Autonomous Command");
    // autoChooser.addOption("Trench Run", Constants.TRENCH_RUN);

    autoChooser.addOption("Galactic Search A","Galactic Search A");
    autoChooser.addOption("Galactic Search B","Galactic Search B");
    autoChooser.addOption("Slalom","Slalom");
    autoChooser.addOption("Barrel","Barrel");
    autoChooser.addOption("Bounce","Bounce");
    autoChooser.setName("AutoChooserTest");

    pushBotChooser.setDefaultOption("PUSH", Constants.PUSH);
    pushBotChooser.setName("Push Ally Bot");

    SmartDashboard.putData(autoChooser);
    SmartDashboard.putData(pushBotChooser);
    drivetrain.resetOdometry();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
    drivetrain.resetOdometry();
    // limelight.setLED(1);
    // limelight.setCAM(0);
  }

  @Override
  public void disabledPeriodic() {
    // limelight.setLED(1);
    // climber.stop();
    // feeder.stop();
    // intake.stopFunnel();
    // intake.stopIntake();
    // intake.stopLowerTower();
    // shooter.stopShooter();
    // intake.intakeUp();
    // climber.withdraw();

        // Display values to driver station
        readTalonsAndShowValues();
  }

  @Override
  public void autonomousInit() {
    CommandScheduler.getInstance().cancelAll();
    drivetrain.resetOdometry();
    // limelight.setLED(1);
    // limelight.setCAM(0);
    
    // shooter.setSpeed(5000);
    // turret.zeroTurret();
    desiredAutoCommand = getDesiredAutoCommand();
    if (desiredAutoCommand != null) {
      desiredAutoCommand.schedule();
    }
    //stopAtCollision.schedule();
  }

  @Override
  public void autonomousPeriodic() {
    CommandScheduler.getInstance().run();
    
  }

  public CommandBase getDesiredAutoCommand() {
    String selected = autoChooser.getSelected();
    String pushSelect = pushBotChooser.getSelected();
    boolean push = false;
    if(pushSelect.equals(Constants.PUSH)){
      push = true;
    }
    double speed = -.3;
    double driveTime = 0.5;
    double shootTime = 3.0;
    waitTime = waitNT.getDouble(0.0);
    // if (selected.equals(Constants.SHOOT_SCOOT)) {
    //   return new SimpleAuto(shooter, drivetrain, speed, shootTime, driveTime, feeder, intake, push);
    // } else if (selected.equals(Constants.TRENCH_RUN)) {
    //   return new SimpleAuto2(shooter, turret, drivetrain, speed, driveTime, shootTime, feeder, intake, push, waitTime);
    // }

    if (selected.equals("Galactic Search A")) {
      if (sideNT.equals("Left")){
        String blueA1 = "paths/goingBall1.wpilib.json";
        String blueA2 = "paths/retrievingBall1.wpilib.json";
        String blueA3 = "paths/goingBall2.wpilib.json";
        String blueA4 = "paths/retrievingBall2.wpilib.json";
        String blueA5 = "paths/goingBall3.wpilib.json";
        String blueA6 = "paths/retrievingBall3.wpilib.json";
        String blueA7 = "paths/ends.wpilib.json";
        String[] trajectoryGroup = {blueA1, blueA2, blueA3, blueA4, blueA5, blueA6, blueA7};
      
        m_robotContainer.getStartAutonomousCommand(blueA1);
        // new IntakeDown(m_robotContainer.intake).schedule();
        // new FeederRunAuto(m_robotContainer.feeder, m_robotContainer.intake, m_robotContainer.shooter, gamepad).schedule();
        for(int i = 1; i < trajectoryGroup.length; i++) {
         m_robotContainer.getAutonomousCommand(trajectoryGroup[i],i).schedule();
        }
      } else {
        String redA1 = "paths/approachingBall1.wpilib.json";
        String redA2 = "paths/receivingBall1.wpilib.json";
        String redA3 = "paths/approachingBall2.wpilib.json";
        String redA4 = "paths/receivingBall2.wpilib.json";
        String redA5 = "paths/approachingBall3.wpilib.json";
        String redA6 = "paths/receivingBall3.wpilib.json";
        String redA7 = "paths/redfinish.wpilib.json";
        String[] trajectoryGroup = {redA1, redA2, redA3, redA4, redA5, redA6, redA7};
        m_robotContainer.getStartAutonomousCommand(redA1);
        // new IntakeDown(m_robotContainer.intake).schedule();
        // new FeederRunAuto(m_robotContainer.feeder, m_robotContainer.intake, m_robotContainer.shooter, gamepad).schedule();
        for(int i = 1; i < trajectoryGroup.length; i++) {
         m_robotContainer.getAutonomousCommand(trajectoryGroup[i],i).schedule();
        }
      }

    } else if (selected.equals("Galactic Search B")){

      if (sideNT.equals("Left")){          
        String blueB1 = "paths/Pathweaver/ryan/GalacticSearchB/Paths/output/headingBall1.wpilib.json";
        String blueB2 = "paths/Pathweaver/ryan/GalacticSearchB/Paths/output/gettingBall1.wpilib.json";
        String blueB3 = "paths/Pathweaver/ryan/GalacticSearchB/Paths/output/headingBall2.wpilib.json";   
        String blueB4 = "paths/Pathweaver/ryan/GalacticSearchB/Paths/output/gettingBall2.wpilib.json";
        String blueB5 = "paths/Pathweaver/ryan/GalacticSearchB/Paths/output/headingBall3.wpilib.json";
        String blueB6 = "paths/Pathweaver/ryan/GalacticSearchB/Paths/output/gettingBall3.wpilib.json";
        String blueB7 = "paths/Pathweaver/ryan/GalacticSearchB/Paths/output/ending.wpilib.json";
        String[] trajectoryGroup = {blueB1, blueB2, blueB3, blueB4, blueB5, blueB6, blueB7};
        m_robotContainer.getStartAutonomousCommand(blueB1);
        // new IntakeDown(m_robotContainer.intake).schedule();
        // new FeederRunAuto(m_robotContainer.feeder, m_robotContainer.intake, m_robotContainer.shooter, gamepad).schedule();
        for(int i = 1; i < trajectoryGroup.length; i++) {
            m_robotContainer.getAutonomousCommand(trajectoryGroup[i],i).schedule();
            
        }
      } else {
        String redB1 = "paths/Pathweaver/ryan/GalacticSearchB/Paths/output/towardsBall1.wpilib.json";
        String redB2 = "paths/Pathweaver/ryan/GalacticSearchB/Paths/output/collectBall1.wpilib.json";
        String redB3 = "paths/Pathweaver/ryan/GalacticSearchB/Paths/output/towardsBall2.wpilib.json";
        String redB4 = "paths/Pathweaver/ryan/GalacticSearchB/Paths/output/collectBall2.wpilib.json";
        String redB5 = "paths/Pathweaver/ryan/GalacticSearchB/Paths/output/towardsBall3.wpilib.json";
        String redB6 = "paths/Pathweaver/ryan/GalacticSearchB/Paths/output/collectBall3.wpilib.json";
        String redB7 = "paths/Pathweaver/ryan/GalacticSearchB/Paths/output/toEnd.wpilib.json";
        // Trajectory
        String[] trajectoryGroup = {redB1, redB2, redB3, redB4, redB5, redB6, redB7};
        m_robotContainer.getStartAutonomousCommand(redB1);
        
        for(int i = 1; i < trajectoryGroup.length; i++) {
         m_robotContainer.getAutonomousCommand(trajectoryGroup[i],i).schedule();
        }
      }

    } else if (selected.equals("Bounce")) {

      m_robotContainer.getBounceAutonomousCommand().schedule();
    } else if (selected.equals("Slalom")) {

      m_robotContainer.getSlalomAutonomousCommand().schedule();
    } else if (selected.equals("Barrel")){

      m_robotContainer.getBarrelAutonomousCommand().schedule();
    }

    return null;
  }

  @Override
  public void teleopInit() {
    drivetrain.resetOdometry();
    CommandScheduler.getInstance().cancelAll();
    // limelight.setLED(1);
    // limelight.setCAM(0);
    
    drivetele.schedule();
    // moveTurret.schedule();
    stopAtCollision.schedule();
  }

  @Override
  public void teleopPeriodic() {
    //This should fire off commands to the robot based on the user input to controller?
    //Every 20 ms
    CommandScheduler.getInstance().run();
    // if(gamepad.getRawButton(Constants.Right_Trigger_Button) && !gamepad.getRawButton(Constants.Right_Bumper_Button)){
    //   intake.runIntakeIn(true);
    // }else if(gamepad.getRawButton(Constants.Right_Bumper_Button) &&! gamepad.getRawButton(Constants.Right_Trigger_Button) ){
    //   intake.runIntakeOut(true);
    // }else{
    //   intake.stopIntake();
    // }

    // if(gamepad.getRawButton(Constants.Left_Bumper_Button)){
    //   // if(shooter.isRunning())
    //   // {
    //   //     shooter.stopShooter();
    //   //     shooter.setRunning(false);
    //   // }else{
    //   //     shooter.setRunning(true);
    //   //     shooter.getToSpeed();
    //   // }
    //   shooter.setRunning(true);
    //   shooter.getToSpeed();
    // } else {
    //   shooter.stopShooter();
    //   shooter.setRunning(false);
    // }

    // if (gamepad.getRawButton(Constants.Right_Joystick_Pressed)) {
    //   turret.targetEntity();
    // }else{
    //   // limelight.setLED(1);
    //   // limelight.setCAM(1);
    // }

    // if(gamepad.getRawButton(Constants.X_Button)){
    //   intake.intakeDown();
    // }
    // if(gamepad.getRawButton(Constants.Y_Button)){
    //   intake.intakeUp();
    // }
    // if(gamepad.getRawButton(Constants.Left_Trigger_Button) || (gamepad.getRawButton(Constants.Right_Bumper_Button) && !gamepad.getRawButton(Constants.Right_Trigger_Button))){
    //   FeederRun run = new FeederRun(feeder, intake, shooter, gamepad);
    //   run.feed();
    // }else{
    //   feeder.stop();
    //   intake.stopFunnel();
    //   intake.stopLowerTower();
    // }
    // if ((gamepad.getPOV() == Constants.D_Pad_Up) && (!rightStick.getRawButton(Constants.RS_Shift_Switch))) {
    //   if (!climber.getDeployed()) {
    //       climber.deploy();
    //   }
    // }
    // if(gamepad.getPOV() == Constants.D_Pad_Down){
    //   intake.runAllOut();
    //   shooter.reverseShooters();
    //   feeder.reverse();
    // }
    // if(gamepad.getRawButton(Constants.Left_Joystick_Pressed)){
      
    //   double speed = gamepad.getRawAxis(Joystick.AxisType.kY.value);
    //   if(speed < 0.0)
    //       speed = 0.0;
    //   climber.climb(speed);
    // } else {
    //   climber.climb(0.0);
    // }

    //Below would be the check to see if the user is saying rotate the color wheel
    if (false) {
      colorWheel.spin();
    }

    if(gamepad.getPOV() == Constants.D_Pad_Left){
      double tempSpeed = shooter.getSpeed() - 100;
      if (tempSpeed < 0) {
        tempSpeed = 0;
      }
      shooter.setSpeed(tempSpeed);
      if(shooter.isRunning()) {
        shooter.getToSpeed();
      }
    } else if(gamepad.getPOV() == Constants.D_Pad_Right){
      double tempSpeed = shooter.getSpeed() +100;
      if (tempSpeed > 6300) {
        tempSpeed = 6300;
      }
      shooter.setSpeed(tempSpeed);
      if(shooter.isRunning()) {
        shooter.getToSpeed();
      }
    }

    // Display values to driver station
    readTalonsAndShowValues();
  }

  @Override
  public void testInit() {
    // limelight.setLED(1);
    // limelight.setCAM(0);
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }

  public void readTalonsAndShowValues() {
    // Display shooter motor speeds
    // SmartDashboard.putNumber("shooter1RPM:", shooter.getLeftRPM());
    // SmartDashboard.putNumber("shooter2RPM:", shooter.getRightRPM());
    // SmartDashboard.putNumber("Shooter Target Speed", shooter.getSpeed());

    // intake.checkLowerTower();

    drivetrain.checkmotors();
    SmartDashboard.putNumber("NavX heading cos", RobotContainer.navx.getRotation2d().getCos());
    SmartDashboard.putNumber("NavX PID", RobotContainer.navx.pidGet());
    SmartDashboard.putNumber("NavX angle", RobotContainer.navx.getAngle());
    SmartDashboard.putString("DriveTrain get pose", drivetrain.getPose().toString());
    //Turret Position
    // SmartDashboard.putNumber("TurretPos", turret.getPosition());
    
    // //Ball Present SensorsS
    // SmartDashboard.putBoolean("Lower Tower Ball Present", intake.getBallPresent());
    // SmartDashboard.putBoolean("High Tower Ball Present", feeder.getBallPresent());

    //Limelight
    // SmartDashboard.putBoolean("Limelight has target", limelight.hasTarget());
    // SmartDashboard.putNumber("Target Distance (ft)", limelight.getTargetDistance());

  }
}
