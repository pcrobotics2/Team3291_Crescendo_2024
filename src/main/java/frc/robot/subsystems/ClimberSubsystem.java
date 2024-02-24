// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {
  /** Creates a new ClimberSubsystem. */

  public CANSparkMax leftclimber;
  public CANSparkMax rightclimber;
  
  public ClimberSubsystem() {
    
    this.leftclimber = new CANSparkMax(Constants.leftClimberID, MotorType.kBrushless);//
    this.rightclimber = new CANSparkMax(Constants.rightClimberID, MotorType.kBrushless);//
   
  }
   
  @Override
  public void periodic() {
  }

public void setClimberIndividual(double positiveSpeed, double negativeSpeed) {
 leftclimber.set(-positiveSpeed); 
 rightclimber.set(-negativeSpeed);
}
public void setClimberTogether(double positiveSpeed, double negativeSpeed) {
 leftclimber.set(positiveSpeed - negativeSpeed); 
 rightclimber.set(-1 * (positiveSpeed - negativeSpeed));
}

public void stop() {
 leftclimber.set(0);
 rightclimber.set(0);
    // TODO Auto-generated method stub
}
}
