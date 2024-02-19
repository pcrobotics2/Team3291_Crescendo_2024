// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DriveCommands;
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Swerve;
import frc.robot.commands.SwerveDrive;
import frc.robot.subsystems.SwerveSubsystem;

public class DriveToApriltag extends Command {
  /** Creates a new DriveToApriltag. */
  VisionSubsystem visionSubsystem;
  SwerveSubsystem swerveSubsystem;
  public DriveToApriltag(VisionSubsystem visionSubsystem, SwerveSubsystem swerveSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.visionSubsystem = visionSubsystem;
    this.swerveSubsystem = swerveSubsystem;
    addRequirements(visionSubsystem, swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double aim = visionSubsystem.proportionalAiming();
    visionSubsystem.drive(aim);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
