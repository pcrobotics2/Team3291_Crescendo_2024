// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Constants;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class ClimberSubsystem extends SubsystemBase {
  /** Creates a new ClimberSubsystem. */

  


  public CommandJoystick controller5 = new CommandJoystick(0);
  public CANSparkMax leftclimber;
  public CANSparkMax rightclimber;
  
  public ClimberSubsystem() {
    
    this.leftclimber = new CANSparkMax(Constants.leftClimberID, MotorType.kBrushless);//
    this.rightclimber = new CANSparkMax(Constants.rightClimberID, MotorType.kBrushless);//
   
   
  }
   
  @Override
  public void periodic() {

  
  }
public void setGoal(){

}

public void setClimber(double positiveSpeed, double negativeSpeed) {
 leftclimber.set(positiveSpeed); 
 rightclimber.set(negativeSpeed);


}

public void stop() {
 leftclimber.set(0);
 rightclimber.set(0);
    // TODO Auto-generated method stub
}


}
