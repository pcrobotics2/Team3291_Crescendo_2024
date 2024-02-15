// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;

public class IntakeMotorSubsystem extends SubsystemBase {
  /** Creates a new IntakeMotorSubsystem. */
    public CANSparkMax intakeMotor;
    public IntakeMotorSubsystem intakeMotorSubsystem;

  public IntakeMotorSubsystem() {
    this.intakeMotor = new CANSparkMax(Constants.intake.IntakeID, CANSparkLowLevel.MotorType.kBrushless); 

  }
  public void moveIntakeMotor (double speed) {
    intakeMotor.set(speed);
  }

//   public Command TestStartEndCommand(double speed) {
//   // implicitly require `this`
//   // return new FunctionalCommand(    // Reset encoders on command start
//   // // Start driving forward at the start of the command
//   // () -> this.moveIntakeMotor(speed),
//   // // Stop driving at the end of the command
//   // interrupted -> this.moveIntakeMotor(0),
//   // // End the command when timer exceeds value
//   // () -> Timer.getFPGATimestamp(),
//   // // Require the drive subsystem
//   // this);


  
// }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
