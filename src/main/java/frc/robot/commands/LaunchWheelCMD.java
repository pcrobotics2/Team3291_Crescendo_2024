// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.LauncherSub;

public class LaunchWheelCMD extends Command {
  /** Creates a new LaunchWheelCMD. */
  LauncherSub launcherSub;
  double speed;
  public LaunchWheelCMD(LauncherSub launcherSub) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.launcherSub = launcherSub;
    addRequirements(launcherSub);
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    launcherSub.setLaunchWheels(-Constants.launchSpeed, Constants.launchSpeed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // this.speed += Constants.intake.launchNoteTimeInSecs * Constants.launcherTargetVoltage / 50;
    // if (this.speed >= Constants.launcherTargetVoltage) {
    //   this.speed = Constants.launcherTargetVoltage;
    // }
    // launcherSub.setLaunchWheelsVoltage(-this.speed, this.speed);
    launcherSub.setLaunchWheels(-Constants.launchSpeed, Constants.launchSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    launcherSub.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
