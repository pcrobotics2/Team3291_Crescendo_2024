// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
 

public class ClimberSubsystem extends SubsystemBase {
  /** Creates a new ClimberSubsystem. */

  public CANSparkMax leftclimber;
  //public CANSparkMax rightclimber;
  
  public ClimberSubsystem() {
    this.leftclimber = new CANSparkMax(20, MotorType.kBrushless);
    //this.rightclimber = new CANSparkMax(0, MotorType.kBrushless);
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
public void setleftClimber(double speed) {
 leftclimber.set(speed);
}
// public void setrightClimber(double speed){
//   rightclimber.set(-speed);
// }

  

  public void stop() {
  leftclimber.set(0);
  //rightclimber.set(0);
    // TODO Auto-generated method stub
}
}
