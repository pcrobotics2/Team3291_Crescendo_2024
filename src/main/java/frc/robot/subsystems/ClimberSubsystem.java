// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;


public class ClimberSubsystem extends SubsystemBase {
  /** Creates a new ClimberSubsystem. */

  public CANSparkMax leftclimber;
  public CANSparkMax rightclimber;
  private static double kDt = 0.02;
  private static double kMaxVelocity = 1.75;
  private static double kMaxAcceleration = 0.75;
  private static double kP = 0.0;
  private static double kI = 0.0;
  private static double kD = 0.0;
  private static double kS = 1.1;
  private static double kG = 1.2;
  private static double kV = 1.3;
  private final Encoder Encoder = new Encoder(1, 2);

  
  public ClimberSubsystem() {
    this.leftclimber = new CANSparkMax(1, MotorType.kBrushless);//
    this.rightclimber = new CANSparkMax(0, MotorType.kBrushless);//
   final TrapezoidProfile.Constraints m_constraints =
    new TrapezoidProfile.Constraints(kMaxVelocity, kMaxAcceleration);
final ProfiledPIDController leftclimbController =
    new ProfiledPIDController(kP, kI, kD, m_constraints, kDt);
final ElevatorFeedforward feedforward = new ElevatorFeedforward(kS, kG, kV);
    
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
public void setClimber(double speed) {
 leftclimber.setVoltage(speed); 
 rightclimber.set(-speed);
}

public void stop() {
 leftclimber.set(0);
 rightclimber.set(0);
    // TODO Auto-generated method stub
}
}
