// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeMotorSubsystem extends SubsystemBase {
  /** Creates a new IntakeMotorSubsystem. */
    public CANSparkMax intakeMotor;

  public IntakeMotorSubsystem() {
    this.intakeMotor = new CANSparkMax(Constants.intake.IntakeID, CANSparkLowLevel.MotorType.kBrushless); 

  }
  public void moveIntakeMotor (double positiveSpeed, double negativeSpeed) {
    intakeMotor.set(positiveSpeed - negativeSpeed);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
