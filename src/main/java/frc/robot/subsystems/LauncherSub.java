// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LauncherSub extends SubsystemBase {
  /** Creates a new launcher. */
  //motor up is higher than motor down
  public CANSparkMax leftLauncher;
  public CANSparkMax rightLauncher;

  public LauncherSub() {
  this.rightLauncher = new CANSparkMax(Constants.rightLauncherID, MotorType.kBrushless); 
  this.leftLauncher = new CANSparkMax(Constants.leftLauncherID, MotorType.kBrushless);

  }

  public void setLaunchWheelUp(double speed) {
    rightLauncher.set(speed);

  }
    public void setLaunchWheels(double rightSpeed, double leftSpeed) {
    rightLauncher.set(rightSpeed);
    leftLauncher.set(leftSpeed);
  }
  public void setLaunchWheelDown(double speed) {
    leftLauncher.set(speed);
  }
  public void setLaunchWheelsVoltage (double rightSpeed, double leftSpeed) {
    rightLauncher.setVoltage(rightSpeed);
    leftLauncher.setVoltage(leftSpeed);
  }
  public void stop() {
    leftLauncher.set(0);
    rightLauncher.set(0);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
