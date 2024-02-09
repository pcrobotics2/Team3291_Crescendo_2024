// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ClimbCMD;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.FeedWheelCMD;
import frc.robot.commands.LaunchWheelCMD;
import frc.robot.commands.SwerveDrive;
import frc.robot.commands.Auto.MildAuto;
import frc.robot.commands.IntakeCMDS.AmpCMD;
import frc.robot.commands.IntakeCMDS.GroundCMD;
import frc.robot.commands.IntakeCMDS.SourceCMD;
import frc.robot.commands.IntakeCMDS.StowCMD;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeMotorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSub;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.ClimberSubsystem;



/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

private final SendableChooser<Command> autoChooser;

  //careful setting the port for controller
  public CommandJoystick controller0 = new CommandJoystick(0);
  public CommandJoystick controller1 = new CommandJoystick(1);
  public LauncherSub launcherSub = new LauncherSub();
  public ClimberSubsystem climberSubsystem = new ClimberSubsystem();
  public IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  public IntakeMotorSubsystem intakeMotorSubsystem = new IntakeMotorSubsystem();
  public FeedWheelCMD feedWheelCMD = new FeedWheelCMD(launcherSub);
  public LaunchWheelCMD launchWheelCMD = new LaunchWheelCMD(launcherSub);
  // public ClimbCMD climbCMD = new ClimbCMD(climberSubsystem);
  public StowCMD stowCMD = new StowCMD(intakeSubsystem);
  public AmpCMD ampCMD = new AmpCMD(intakeSubsystem);
  public GroundCMD groundCMD = new GroundCMD(intakeSubsystem);
  public SourceCMD sourceCMD = new SourceCMD(intakeSubsystem);
  // The robot's subsystems and commands are definelad here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  // private final CommandXboxController m_driverController =
  //    new CommandXboxController(OperatorConstants.kDriverControllerPort);
  public final JoystickButton robotCentricButton = new JoystickButton(controller0.getHID(), Constants.buttonList.lb);

  //subsystems\\
  private SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings

    configureBindings();

    // controller5.button(Constants.buttonList.b).whileTrue(launchWheelCMD);
    // controller5.button(Constants.buttonList.x).whileTrue(feedWheelCMD);
   
   
    // controller5.button(Constants.buttonList.x).whileTrue(climbCMD);

    //intake
    controller0.button(Constants.buttonList.a).toggleOnTrue(stowCMD);
    controller0.button(Constants.buttonList.b).toggleOnTrue(groundCMD);
    controller0.button(Constants.buttonList.x).toggleOnTrue(sourceCMD);
    controller0.button(Constants.buttonList.y).toggleOnTrue(ampCMD);

    //Autonomous
  autoChooser = AutoBuilder.buildAutoChooser();

  SmartDashboard.putData("Auto Chooser", autoChooser);
    controller0.povDown().toggleOnTrue(ampCMD);
    controller0.povUp().toggleOnTrue(stowCMD);
    controller0.povDown().toggleOnTrue(ampCMD);
    controller0.povUp().toggleOnTrue(stowCMD);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));

    swerveSubsystem.setDefaultCommand(
      new SwerveDrive(
        swerveSubsystem,
        () -> controller0.getRawAxis(1),
        () -> controller0.getRawAxis(0),
        () -> controller0.getRawAxis(4),
        () -> robotCentricButton.getAsBoolean()
      )
    );
    climberSubsystem.setDefaultCommand(
      new ClimbCMD(
        climberSubsystem,
        () -> controller0.getRawAxis(2),
        () -> controller0.getRawAxis(3)
      )
    );
  
    
    

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    // m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());

    SmartDashboard.putData("Test Auto", new PathPlannerAuto("Test Auto"));

  }
                                                                                             
  public Command getAutonomousCommand() {
    // TODO Auto-generated method stub
    
    return new PathPlannerAuto("Test Auto");
    //return autoChooser.getSelected();
    //return new MildAuto(swerveSubsystem);
  }

  
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
   
}
