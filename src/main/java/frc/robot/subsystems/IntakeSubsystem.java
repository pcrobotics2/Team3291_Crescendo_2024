// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */

  public DutyCycleEncoder intakeEncoder;
  public DigitalInput intakeLimitSwitch;
  public PIDController pidController;
  public CANSparkMax pivotMotor;
  public CANSparkMax intakeMotor;


  public IntakeSubsystem() {
    this.intakeEncoder = new DutyCycleEncoder(Constants.intake.encoderID);
    this.intakeLimitSwitch = new DigitalInput(Constants.intake.intakeLimitSwitchID);

    this.pidController = new PIDController(Constants.intake.intakePID.kp, Constants.intake.intakePID.ki, Constants.intake.intakePID.kd);

    this.pivotMotor = new CANSparkMax(Constants.intake.PivotID, CANSparkLowLevel.MotorType.kBrushless);
    this.intakeMotor = new CANSparkMax(Constants.intake.IntakeID, CANSparkLowLevel.MotorType.kBrushless); 

  }

 


    // Input: Desired state
    PivotTarget pivot_target = PivotTarget.STOW;
    IntakeState intake_state = IntakeState.NONE;

    // Output: Motor set values
    //double intake_pivot_voltage = 0.0;
    double intake_speed = 0.0;
  


  public enum PivotTarget {
    NONE,
    GROUND,
    SOURCE,
    AMP,
    STOW
  }
  public enum IntakeState {
    NONE,
    INTAKE,
    EJECT,
    PULSE,
    SHOOTER
  }
   

  public double giveVoltage(double pivot_angle) {
    // Pivot control
    //double pivot_angle = pivotTargetToAngle(pivot_target);
    double intake_pivot_voltage = pidController.calculate(getPivotAngleDegrees(), pivot_angle);

    // If the pivot is at exactly 0.0, it's probably not connected, so disable it
    if (intakeEncoder.get() == 0.0) {
      intake_pivot_voltage = 0.0;
    }
    if (intake_pivot_voltage > 10) {
      intake_pivot_voltage = 10;
    }
    if (intake_pivot_voltage < -10) {
      intake_pivot_voltage = -10;
    }
    SmartDashboard.putNumber("intake_pivot_voltage", intake_pivot_voltage);
    return intake_pivot_voltage;
  }
/* 
  @Override
  public void writePeriodicOutputs() {
    mPivotMotor.setVoltage(m_periodicIO.intake_pivot_voltage);

    mIntakeMotor.set(m_periodicIO.intake_speed);
  }
*/
  
  public void stopIntake() {
    intakeMotor.set(0);
    pivotMotor.set(0);
  }


  public double pivotTargetToAngle(PivotTarget target) {
    switch (target) {
      case GROUND:
        return Constants.intake.groundAngle;
      case SOURCE:
        return Constants.intake.sourceAngle;
      case AMP:
        return Constants.intake.ampAngle;
      case STOW:
        return Constants.intake.stowAngle;
      default:
        // "Safe" default
        return 180;
    }
  }

/*   public IntakeState getIntakeState() {
    return m_periodicIO.intake_state;
  }
*/
  public double getPivotAngleDegrees() {
    SmartDashboard.putNumber("encoder", intakeEncoder.getAbsolutePosition());
    double value = intakeEncoder.getAbsolutePosition() -
        Constants.intake.k_pivotEncoderOffset + 0.5;


    value *= 360;
    SmartDashboard.putNumber("value", value);
    return value;
  }

  public boolean getIntakeHasNote() {
    // NOTE: this is intentionally inverted, because the limit switch is normally
    // closed
    return !intakeLimitSwitch.get();
  }
   
  public void goToGround() {
    pivot_target = PivotTarget.GROUND;
    double pivot_angle = pivotTargetToAngle(pivot_target);
    SmartDashboard.putNumber("getVoltage", giveVoltage(pivot_angle));
    SmartDashboard.putNumber("encoder", intakeEncoder.getAbsolutePosition());

    pivotMotor.setVoltage(giveVoltage(pivot_angle));
  }
  public void goToSource() {
    pivot_target = PivotTarget.SOURCE;
    double pivot_angle = pivotTargetToAngle(pivot_target);
    SmartDashboard.putNumber("getVoltage", giveVoltage(pivot_angle));
    SmartDashboard.putNumber("encoder", intakeEncoder.getAbsolutePosition());
    pivotMotor.setVoltage(giveVoltage(pivot_angle));
  }
  public void goToAmp() {
    pivot_target = PivotTarget.AMP;
    double pivot_angle = pivotTargetToAngle(pivot_target);
    SmartDashboard.putNumber("getVoltage", giveVoltage(pivot_angle));
    SmartDashboard.putNumber("encoder", intakeEncoder.getAbsolutePosition());
    pivotMotor.setVoltage(giveVoltage(pivot_angle));
  }
  public void goToStow() {
    pivot_target = PivotTarget.STOW;
    double pivot_angle = pivotTargetToAngle(pivot_target);
    SmartDashboard.putNumber("getVoltage", giveVoltage(pivot_angle));
    SmartDashboard.putNumber("encoder", intakeEncoder.getAbsolutePosition());
    pivotMotor.setVoltage(giveVoltage(pivot_angle));
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
