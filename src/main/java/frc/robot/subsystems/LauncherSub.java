// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LauncherSub extends SubsystemBase {
  /** Creates a new launcher. */
  //motor up is higher than motor down
  public CANSparkMax upLauncher;
  public CANSparkMax downLauncher;

  public LauncherSub() {
    this.upLauncher = new CANSparkMax(7, MotorType.kBrushless);
    this.downLauncher = new CANSparkMax(14, MotorType.kBrushless);

    upLauncher.follow(downLauncher);
  }

  // public Command getIntakeCommand() {
  //   return this.startEnd(
  //     () -> {
  //       setFeedWheel(kIntakeFeederSpeed);
  //       setLaunchWheel(kIntakeLauncherSpeed);
  //     }, 
  //     () -> 
  //       stop();
  //     });
  // }
  public void setLaunchWheel(double speed) {
    downLauncher.set(speed);
  }
  public void setFeedWheel(double speed) {
    downLauncher.set(speed);
  }
  public void stop() {
    downLauncher.set(0);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
