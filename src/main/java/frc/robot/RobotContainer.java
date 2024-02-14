// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.Swerve;
import frc.robot.commands.ClimbCMD;
import frc.robot.commands.FeedWheelCMD;
import frc.robot.commands.LaunchWheelCMD;
import frc.robot.commands.SwerveDrive;
import frc.robot.commands.Auto.MildAuto;
import frc.robot.commands.IntakeCMDS.AmpCMD;
import frc.robot.commands.IntakeCMDS.GroundCMD;
import frc.robot.commands.IntakeCMDS.SourceCMD;
import frc.robot.commands.IntakeCMDS.StowCMD;
import frc.robot.commands.IntakeCMDS.IntakeMotor.EjectCMD;
import frc.robot.commands.IntakeCMDS.IntakeMotor.IntakeMotorCMD;
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
  public CommandJoystick controller1 = new CommandJoystick(0); //same for testing
  public LauncherSub launcherSub = new LauncherSub();
  public ClimberSubsystem climberSubsystem = new ClimberSubsystem();
  public IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  public IntakeMotorSubsystem intakeMotorSubsystem = new IntakeMotorSubsystem();
  //public SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  public FeedWheelCMD feedWheelCMD = new FeedWheelCMD(launcherSub);
  public LaunchWheelCMD launchWheelCMD = new LaunchWheelCMD(launcherSub);
  public StowCMD stowCMD = new StowCMD(intakeSubsystem);
  public AmpCMD ampCMD = new AmpCMD(intakeSubsystem);
  public GroundCMD groundCMD = new GroundCMD(intakeSubsystem);
  public SourceCMD sourceCMD = new SourceCMD(intakeSubsystem);
  public EjectCMD ejectCMD = new EjectCMD(intakeMotorSubsystem);
  public IntakeMotorCMD intakeMotorCMD = new IntakeMotorCMD(intakeMotorSubsystem); 
  
  public final JoystickButton robotCentricButton = new JoystickButton(controller0.getHID(), Constants.buttonList.lb);
  public final JoystickButton aToggleButton = new JoystickButton(controller0.getHID(), Constants.buttonList.a);


  //subsystems\\
  private SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
 // Subsystem initialization
        // intakeMotorSubsystem = new IntakeMotorSubsystem();

        // // Register Named Commands
        // NamedCommands.registerCommand("test", intakeMotorSubsystem.TestStartEndCommand(-0.1));
        // NamedCommands.registerCommand("testStop", intakeMotorSubsystem.TestStartEndCommand(0.5));
        NamedCommands.registerCommand("EjectCMD", new EjectCMD(intakeMotorSubsystem).withTimeout(1));
        NamedCommands.registerCommand("IntakeMotorCMD", new IntakeMotorCMD(intakeMotorSubsystem).withTimeout(1));
        NamedCommands.registerCommand("LaunchWheelCMD", new LaunchWheelCMD(launcherSub).withTimeout(1));
        NamedCommands.registerCommand("FeedWheelCMD", new FeedWheelCMD(launcherSub).withTimeout(1));
        NamedCommands.registerCommand("AmpCMD", new AmpCMD(intakeSubsystem).until(intakeSubsystem::ampAtAngle));
        NamedCommands.registerCommand("SourceCMD", new SourceCMD(intakeSubsystem).until(intakeSubsystem::sourceAtAngle));
        NamedCommands.registerCommand("GroundCMD", new GroundCMD(intakeSubsystem).until(intakeSubsystem::groundAtAngle));
        NamedCommands.registerCommand("StowCMD", new StowCMD(intakeSubsystem).until(intakeSubsystem::stowAtAngle));




    configureBindings();

    controller0.button(Constants.buttonList.b).whileTrue(launchWheelCMD);
    controller0.button(Constants.buttonList.x).whileTrue(feedWheelCMD);
    
    controller1.button(Constants.buttonList.b).whileTrue(launchWheelCMD);
    controller1.button(Constants.buttonList.x).whileTrue(feedWheelCMD);
   
    //intake

    controller1.povDown().whileTrue(groundCMD);
    controller1.povUp().whileTrue(stowCMD);
    controller1.povLeft().whileTrue(sourceCMD);
    controller1.povRight().whileTrue(ampCMD);

    controller1.button(Constants.buttonList.rb).toggleOnTrue(ejectCMD);
    controller1.button(Constants.buttonList.lb).toggleOnTrue(intakeMotorCMD);

    controller0.button(Constants.buttonList.rb).whileTrue(ejectCMD);
    controller0.button(Constants.buttonList.lb).whileTrue(intakeMotorCMD);
    
    
    //Autonomous
  autoChooser = AutoBuilder.buildAutoChooser();

  SmartDashboard.putData("Auto Chooser", autoChooser);
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
        () -> controller0.getRawAxis(3),
        () -> robotCentricButton.getAsBoolean()
      )
    );
    
    controller0.button(Constants.buttonList.y).whileTrue(
      intakeMotorSubsystem.startEnd(
        ()->{
          intakeMotorSubsystem.moveIntakeMotor(0.3);
        }, 
        ()->{
          intakeMotorSubsystem.moveIntakeMotor(0);
        }
      )
    );

    SmartDashboard.putData("TestAuto", new PathPlannerAuto("TestAuto"));
  }
                                                                                             
  public Command getAutonomousCommand() {
    // TODO Auto-generated method stub
    
    return new PathPlannerAuto("TestMotorAuto");
    //return autoChooser.getSelected();
    //return new MildAuto(swerveSubsystem);
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
}
