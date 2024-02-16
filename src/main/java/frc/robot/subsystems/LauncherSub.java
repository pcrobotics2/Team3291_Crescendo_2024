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
  public CANSparkMax downLauncher;
  public CANSparkMax upLauncher;

  public LauncherSub() {
  this.upLauncher = new CANSparkMax(Constants.upLauncherID, MotorType.kBrushless); //7
  this.downLauncher = new CANSparkMax(Constants.downLauncherID, MotorType.kBrushless);//14

  }

  public void setLaunchWheelUp(double speed) {
    upLauncher.set(speed);

  }
    public void setLaunchWheels(double upSpeed, double downSpeed) {
    upLauncher.set(upSpeed);
    downLauncher.set(downSpeed);
  }
  public void setLaunchWheelDown(double speed) {
    downLauncher.set(speed);
  }
  public void stop() {
    downLauncher.set(0);
    upLauncher.set(0);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
