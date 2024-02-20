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
//import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.ColorChanger;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */

  public DutyCycleEncoder intakeEncoder;
  public DigitalInput intakeLimitSwitch;
  public PIDController pidController;
  public CANSparkMax pivotMotor;
  public ColorChanger colorChanger;
  
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

  // Input: Desired state
  public PivotTarget pivot_target = PivotTarget.STOW;
  public IntakeState intake_state = IntakeState.NONE;

  // Output: Motor set values
  //double intake_pivot_voltage = 0.0;
  public double intake_speed = 0.0;

  public IntakeSubsystem(ColorChanger colorChanger) {
    this.colorChanger = colorChanger;
    this.intakeEncoder = new DutyCycleEncoder(Constants.intake.encoderID);
    this.intakeLimitSwitch = new DigitalInput(Constants.intake.intakeLimitSwitchID);

    this.pidController = new PIDController(Constants.intake.intakePID.kp, Constants.intake.intakePID.ki, Constants.intake.intakePID.kd);
    //this.pidController.enableContinuousInput(0, 360);
    

    this.pivotMotor = new CANSparkMax(Constants.intake.PivotID, CANSparkLowLevel.MotorType.kBrushless);

  }
   

  public double giveVoltage(double pivot_angle, double current_angle) {
    // Pivot control
    SmartDashboard.putNumber("originalAngle", current_angle);
    //double pivot_angle = pivotTargetToAngle(pivot_target);
    
    //double angle = Math.abs(360 - current_angle); //reverses it
    double angle = current_angle;
    SmartDashboard.putNumber("updatedAngle", angle);

    double intake_pivot_voltage = pidController.calculate(angle, pivot_angle);

    // If the pivot is at exactly 0.0, it's probably not connected, so disable it
    SmartDashboard.putNumber("pid output", intake_pivot_voltage);

    //SmartDashboard.putNumber("intake_pivot_voltage", intake_pivot_voltage);
    System.out.println("error" + intake_pivot_voltage);
    //double adjustedIntakePivotVoltage = 10 - Math.abs(intake_pivot_voltage);
    double adjustedIntakePivotVoltage = intake_pivot_voltage; //error reversed for voltage
    if (intakeEncoder.get() == 0.0) {
      adjustedIntakePivotVoltage = 0.0;
    }
    if (adjustedIntakePivotVoltage > 3) {
      adjustedIntakePivotVoltage = 3;
    }
    if (adjustedIntakePivotVoltage < -3) {
      adjustedIntakePivotVoltage = -3;
    }
    System.out.println("division error" + adjustedIntakePivotVoltage);
    return adjustedIntakePivotVoltage;
  }
/* 
  @Override
  public void writePeriodicOutputs() {
    mPivotMotor.setVoltage(m_periodicIO.intake_pivot_voltage);

    mIntakeMotor.set(m_periodicIO.intake_speed);
  }
*/
  
  public void stopIntake() {
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
  // public double getPivotAngleDegrees() {
  //   //SmartDashboard.putNumber("encoder", intakeEncoder.getAbsolutePosition());
  //   System.out.println("encoder position:" + intakeEncoder.getAbsolutePosition());
  //   double value = intakeEncoder.getAbsolutePosition() - Constants.intake.k_pivotEncoderOffset;
  //   value *= 360;
  //   //SmartDashboard.putNumber("angle value", value);
  //   System.out.println("angled from encoder:" + value);
  //   return value;
  // }
  public double getCurrentAngle() {
    double value = intakeEncoder.getAbsolutePosition();
    value *= 360;
    value = value + Constants.intake.k_pivotEncoderOffset;
    if (value > 360) {
      value %= 360;
    }
    return value;
  }
  public boolean getIntakeHasNote() {
    //must be inverted
    return !intakeLimitSwitch.get();
  }
   
  public boolean ampAtAngle() {
    boolean value = false;
    if (getCurrentAngle() < Constants.intake.ampAngle + Constants.angleDeadband && getCurrentAngle() > Constants.intake.ampAngle - Constants.angleDeadband) {
      value = true;
    }
    return value;
  }
  public boolean groundAtAngle() {
    boolean value = false;
    if (getCurrentAngle() < Constants.intake.groundAngle + Constants.angleDeadband && getCurrentAngle() > Constants.intake.groundAngle - Constants.angleDeadband) {
      value = true;
    }
    return value;
  }
  public boolean stowAtAngle() {
    boolean value = false;
    if (getCurrentAngle() < Constants.intake.stowAngle + Constants.angleDeadband && getCurrentAngle() > Constants.intake.stowAngle - Constants.angleDeadband) {
      value = true;
    }
    return value;
  }
  public boolean sourceAtAngle() {
    boolean value = false;
    if (getCurrentAngle() < Constants.intake.sourceAngle + Constants.angleDeadband && getCurrentAngle() > Constants.intake.sourceAngle - Constants.angleDeadband) {
      value = true;
    }
    return value;
  }

  public void goToGround() {

     if (getIntakeHasNote()) {
      goToStow();
    }
    pivot_target = PivotTarget.GROUND;
    pidController.setP(SmartDashboard.getNumber("key", Constants.intake.intakePID.kp));
    double pivot_angle = pivotTargetToAngle(pivot_target);
    System.out.println("stow angle target: " + pivot_angle);
    System.out.println("final voltage: " + giveVoltage(pivot_angle, getCurrentAngle()) + "\n\n");
    double voltage = giveVoltage(pivot_angle, getCurrentAngle());
    pivotMotor.setVoltage(voltage);
    SmartDashboard.putNumber("getVoltage", voltage);
    System.out.println("s");
  }

  public void goToSource() {
    pivot_target = PivotTarget.SOURCE;
    double pivot_angle = pivotTargetToAngle(pivot_target);
    System.out.println("stow angle target: " + pivot_angle);
    System.out.println("final voltage: " + giveVoltage(pivot_angle, getCurrentAngle()) + "\n\n");
    double voltage = giveVoltage(pivot_angle, getCurrentAngle());
    pivotMotor.setVoltage(voltage);
    SmartDashboard.putNumber("getVoltage", voltage);
    System.out.println("s");
  }

  public void goToAmp() {
    pivot_target = PivotTarget.AMP;
    double pivot_angle = pivotTargetToAngle(pivot_target);
    System.out.println("stow angle target: " + pivot_angle);
    System.out.println("final voltage: " + giveVoltage(pivot_angle, getCurrentAngle()) + "\n\n");
    double voltage = giveVoltage(pivot_angle, getCurrentAngle());
    pivotMotor.setVoltage(voltage);
    SmartDashboard.putNumber("getVoltage", voltage);
    System.out.println("s");
  }

  public void goToStow() {
    pivot_target = PivotTarget.STOW;
    double pivot_angle = pivotTargetToAngle(pivot_target);
    System.out.println("stow angle target: " + pivot_angle);
   System.out.println("final voltage: " + giveVoltage(pivot_angle, getCurrentAngle()) + "\n\n");
    double voltage = giveVoltage(pivot_angle, getCurrentAngle());
    pivotMotor.setVoltage(voltage);
    SmartDashboard.putNumber("getVoltage", voltage);
    System.out.println("s");
  }



  @Override
  public void periodic() {
    SmartDashboard.putBoolean("limit switch", getIntakeHasNote());
    //This method will be called once per scheduler run
    if (!getIntakeHasNote() && getCurrentAngle() < Constants.intake.stowAngle + Constants.angleDeadband && getCurrentAngle() > Constants.intake.stowAngle - Constants.angleDeadband) {
      colorChanger.setCOLORWAVESLAVA();
    }
    if (getIntakeHasNote() && getCurrentAngle() < Constants.intake.stowAngle + Constants.angleDeadband && getCurrentAngle() > Constants.intake.stowAngle - Constants.angleDeadband) {
      colorChanger.setLAWNGREEN();
    }
    if (getIntakeHasNote()) {
      colorChanger.setRAINBOWOCEAN();
    }
    SmartDashboard.putNumber("encoder reading", getCurrentAngle());
  }
}
  

