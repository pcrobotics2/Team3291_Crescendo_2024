// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCMDS.IntakeMotor;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeMotorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeMotorCMD extends Command {
  IntakeMotorSubsystem intakeMotorSubsystem;
  IntakeSubsystem intakeSubsystem;
  /** Creates a new IntakeMotorCMD. */
  public IntakeMotorCMD(IntakeMotorSubsystem intakeMotorSubsystem, IntakeSubsystem intakeSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intakeMotorSubsystem = intakeMotorSubsystem;
    this.intakeSubsystem = intakeSubsystem;
    addRequirements(intakeMotorSubsystem, intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}
  

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!intakeSubsystem.getIntakeHasNote()) {
    intakeMotorSubsystem.moveIntakeMotor(-1 * Constants.intake.intakeSpeed);
    }
    if (intakeSubsystem.getIntakeHasNote() && intakeSubsystem.getCurrentAngle() < Constants.intake.groundAngle + Constants.angleDeadband && intakeSubsystem.getCurrentAngle() > Constants.intake.groundAngle - Constants.angleDeadband) {

    }
    // if (!intakeSubsystem.getIntakeHasNote()) {
    // intakeMotorSubsystem.moveIntakeMotor(-1 * Constants.intake.intakeSpeed);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeMotorSubsystem.moveIntakeMotor(0);//stops it
    //ColorChanger.setGOLD();
   // ColorChanger.setGOLD();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}




