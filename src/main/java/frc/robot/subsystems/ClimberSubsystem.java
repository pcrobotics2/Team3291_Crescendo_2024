// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {
  /** Creates a new ClimberSubsystem. */

  public CANSparkMax leftclimber = new CANSparkMax(1, MotorType.kBrushless);
  public CANSparkMax rightclimber = new CANSparkMax(0, MotorType.kBrushless);
  
  public ClimberSubsystem() {
    //leftclimber.set(1);
    //rightclimber.set(-1);
    
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

public void stop() {
 leftclimber.set(0);
 rightclimber.set(0);
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'stop'");
}
}
