// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCMDS;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.subsystems.IntakeSubsystem;

public class MoveIntakeMotorCMD extends Command {
  /** Creates a new MoveIntakeMotorCMD. */
  IntakeSubsystem intakeSubsystem;
  DoubleSupplier positiveSupplier;
  DoubleSupplier negativeSupplier;
  public MoveIntakeMotorCMD(
    IntakeSubsystem intakeSubsystem,
    DoubleSupplier positiveSupplier,
    DoubleSupplier negativeSupplier
  ) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intakeSubsystem = intakeSubsystem;
    addRequirements(intakeSubsystem);

    this.positiveSupplier = positiveSupplier;
    this.negativeSupplier = negativeSupplier;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double positiveSpeed = positiveSupplier.getAsDouble();
    double negativeSpeed = negativeSupplier.getAsDouble();
    intakeSubsystem.moveIntakeMotor(positiveSpeed, negativeSpeed);
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
