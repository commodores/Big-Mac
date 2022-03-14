// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.Constants.OIConstants;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.AppleSauceNumberOne;
import frc.robot.commands.BlueFish;
import frc.robot.commands.ClearHopper;
import frc.robot.commands.ClimberDown;
import frc.robot.commands.ClimberIn;
import frc.robot.commands.ClimberOut;
import frc.robot.commands.ClimberUp;
import frc.robot.commands.CrazyShot;
import frc.robot.commands.DriveManual;
import frc.robot.commands.FireBalls;
import frc.robot.commands.FlashyMove;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.LeeroyJenkins;
import frc.robot.commands.RedFish;
import frc.robot.commands.RunTrajectory;
import frc.robot.commands.ShootHigh;
import frc.robot.commands.ShootLow;
import frc.robot.commands.TwoFish;



/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  //public final DriveTrain m_drivetrain = new DriveTrain();
  public final static DriveTrain m_drivetrain = new DriveTrain();
  public final static Climber m_Climber = new Climber();
  public final static Intake m_intake = new Intake();
  public final static Shooter m_shooter = new Shooter();
  
  public final static XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  public final XboxController m_driver2Controller = new XboxController(OIConstants.kDriverController2Port);
  public final XboxController m_driver3Controller = new XboxController(OIConstants.kDriverController3Port);

  private final SendableChooser<String> m_autoChooser = new SendableChooser<>();
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    /* Initialize various systems on robotInit. */
    this.initializeStartup();

    /* Initialize autonomous command chooser and display on the SmartDashboard. */
    this.initializeAutoChooser();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    //Shooter
    
      new JoystickButton(m_driver2Controller, Button.kLeftBumper.value)
        .whileHeld(new ShootLow());

      new JoystickButton(m_driver2Controller, Button.kRightBumper.value)
      .whileHeld(new ShootHigh());

      new JoystickButton(m_driverController, Button.kB.value)
      .whileHeld(new CrazyShot());
    

    //Intake

      new JoystickButton(m_driver2Controller, Button.kA.value)
        .whileHeld(new IntakeCommand());
      
      new JoystickButton(m_driverController, Button.kA.value)
        .whileHeld(new FireBalls());

      new JoystickButton(m_driver2Controller, Button.kB.value)
        .whileHeld(new ClearHopper());      

      new JoystickButton(m_driver2Controller, Button.kX.value)
        .whenPressed(() -> m_intake.extendIntake());

      new JoystickButton(m_driver2Controller, Button.kY.value)
        .whenPressed(() -> m_intake.retractIntake());        

    //Climber
      
      new JoystickButton(m_driver3Controller, Button.kY.value)
      .whileHeld(new ClimberUp());

      new JoystickButton(m_driver3Controller, Button.kA.value)
      .whileHeld(new ClimberDown());

      new JoystickButton(m_driver3Controller, Button.kX.value)
      .whileHeld(new ClimberOut());

      new JoystickButton(m_driver3Controller, Button.kB.value)
      .whileHeld(new ClimberIn());

      new JoystickButton(m_driver3Controller, Button.kLeftBumper.value)
      .whenPressed(() -> m_Climber.climberLock());

      new JoystickButton(m_driver3Controller, Button.kRightBumper.value)
      .whenPressed(() -> m_Climber.climberUnlock());
    
  }

  private void initializeStartup() {
    m_drivetrain.setDefaultCommand(
      new DriveManual(m_drivetrain));
  }

  /**
   * Set options for autonomous command chooser and display them for selection on the SmartDashboard.
   * Using string chooser rather than command chooser because if using a command chooser, will instantiate
   * all the autonomous commands. This may cause problems (e.g. initial trajectory position is from a
   * different command's path).
   */
  private void initializeAutoChooser()
  {
    /* Add options (which autonomous commands can be selected) to chooser. */
    m_autoChooser.setDefaultOption("Do Nothing", "doNothing");
    m_autoChooser.addOption("Test It", "testPath");
    m_autoChooser.addOption("Drive off Tarmac", "offTarmac");
    m_autoChooser.addOption("1 ball", "ball1");
    m_autoChooser.addOption("2 ball", "ball2");
    m_autoChooser.addOption("3 ball", "ball3");
    m_autoChooser.addOption("4 ball", "ball4");
    m_autoChooser.addOption("5 ball", "ball5");

    /* Display chooser on SmartDashboard for operators to select which autonomous command to run during the auto period. */
    SmartDashboard.putData("Autonomous Command", m_autoChooser);
    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    
    switch (m_autoChooser.getSelected())
    {
      case "testPath" :
        RobotContainer.m_drivetrain.setPos(0, 0);
        RobotContainer.m_drivetrain.zeroSensors();
        return new RunTrajectory("testPath");

      case "offTarmac":
        RobotContainer.m_drivetrain.setPos(0, 0);
        RobotContainer.m_drivetrain.zeroSensors();
        return new FlashyMove();

      case "ball1" :
        RobotContainer.m_drivetrain.setPos(0, 0);
        RobotContainer.m_drivetrain.zeroSensors();
        return new AppleSauceNumberOne();

      case "ball2" :
        RobotContainer.m_drivetrain.setPos(0, 0);
        RobotContainer.m_drivetrain.zeroSensors();
        return new TwoFish();

      case "ball3" :
        RobotContainer.m_drivetrain.setPos(0, 0);
        RobotContainer.m_drivetrain.zeroSensors();
        return new RedFish();

      case "ball4" :
        RobotContainer.m_drivetrain.setPos(0, 0);
        RobotContainer.m_drivetrain.zeroSensors();
        return new BlueFish();

      case "ball5" :
        RobotContainer.m_drivetrain.setPos(0, 0);
        RobotContainer.m_drivetrain.zeroSensors();
        return new LeeroyJenkins();

      default:
        System.out.println("\nError selecting autonomous command:\nCommand selected: " + m_autoChooser.getSelected() + "\n");
        return null;
    }
    
  }
}