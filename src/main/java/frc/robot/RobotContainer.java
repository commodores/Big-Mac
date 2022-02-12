// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
//import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.Constants.OIConstants;
//import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.DriveManual;



/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  //public final DriveTrain m_drivetrain = new DriveTrain();
  public final DriveTrain m_drivetrain = new DriveTrain();
  public final Climber m_Climber = new Climber();
  public final Intake m_intake = new Intake();
  public final Shooter m_shooter = new Shooter();
  public final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  
  public final static XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  public final XboxController m_driver2Controller = new XboxController(OIConstants.kDriverController2Port);

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

    new JoystickButton(m_driverController, Button.kLeftBumper.value)
      .whenPressed(() -> m_shooter.setRPM(400))
      .whenReleased(() -> m_shooter.setRPM(-1));

    new JoystickButton(m_driverController, Button.kRightBumper.value)
      .whenPressed(() -> m_shooter.setRPM(600))
      .whenReleased(() -> m_shooter.setRPM(-1));

    //Intake

    new JoystickButton(m_driverController, Button.kA.value)
      .whileHeld(() -> m_intake.runIntake(-.75))
      .whenReleased(() -> m_intake.stopIntake());

    new JoystickButton(m_driverController, Button.kB.value)
    .whileHeld(() -> m_intake.runIntake(.75))
    .whenReleased(() -> m_intake.stopIntake());

    //Climber

  }

  private void initializeStartup() {
    
    //SmartDashboard.putData("Ramp it up!!", new AutoShoot());
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
    m_autoChooser.addOption("2 Ball Auto", "twoball");
    m_autoChooser.addOption("3 Ball Auto", "threeball");
    m_autoChooser.addOption("5 Ball Auto", "fiveball");

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
      case "threeball":
        //return new SimpleShoot();
        //RobotContainer.m_drivetrain.setPos(Units.feetToMeters(3.5), Units.feetToMeters(-5));
        //return new RunTrajectory(getSlalom());
      case "sixball" :
        //return new SixBallAuto();
      
      case "SecondAuto" :
        //return new DefenseTrench();
    
      default:
        System.out.println("\nError selecting autonomous command:\nCommand selected: " + m_autoChooser.getSelected() + "\n");
        return null;
    }
    
  }
}