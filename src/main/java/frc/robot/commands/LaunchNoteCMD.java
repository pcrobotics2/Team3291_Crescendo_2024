// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeMotorSubsystem;
import frc.robot.subsystems.LauncherSub;

public class LaunchNoteCMD extends Command {
  IntakeMotorSubsystem intakeMotorSubsystem;
  LauncherSub launcherSub;
  double timeCheck;

  /** Creates a new LaunchNoteCMD. */
  public LaunchNoteCMD(IntakeMotorSubsystem intakeMotorSubsystem, LauncherSub launcherSub) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intakeMotorSubsystem = intakeMotorSubsystem;
    this.launcherSub = launcherSub;
    addRequirements(launcherSub, intakeMotorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.timeCheck = Timer.getFPGATimestamp();
    launcherSub.setLaunchWheels(Constants.launchSpeed, -Constants.launchSpeed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Timer.getFPGATimestamp() - timeCheck > Constants.intake.launchNoteTimeInSecs) {
    intakeMotorSubsystem.moveIntakeMotor(Constants.intake.ejectSpeed);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    launcherSub.setLaunchWheels(0, 0);
    intakeMotorSubsystem.moveIntakeMotor(0);//stops it
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
