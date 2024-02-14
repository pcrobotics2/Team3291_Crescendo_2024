package frc.robot.commands;
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;


public class ClimbCMD extends Command {
  /** Creates a new MoveIntakeMotorCMD. */
  ClimberSubsystem climberSubsystem;
  DoubleSupplier positiveSupplier;
  DoubleSupplier negativeSupplier;
  BooleanSupplier aToggle;
  public ClimbCMD(
  ClimberSubsystem climberSubsystem,
    DoubleSupplier positiveSupplier,
    DoubleSupplier negativeSupplier,
    BooleanSupplier aToggle
  ) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.climberSubsystem = climberSubsystem;
    addRequirements(climberSubsystem);

    this.positiveSupplier = positiveSupplier;
    this.negativeSupplier = negativeSupplier;
    this.aToggle = aToggle;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    boolean isAToggled = aToggle.getAsBoolean();
    if (isAToggled) {
      boolean switchIt = true;
    }
    double positiveSpeed = positiveSupplier.getAsDouble();
    double negativeSpeed = negativeSupplier.getAsDouble();
    climberSubsystem.setClimber(positiveSpeed, negativeSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climberSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
