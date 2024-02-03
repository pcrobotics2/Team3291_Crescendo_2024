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
  private static double kDt = 0.02;
  private static double kMaxVelocity = 1.0;
  private static double kMaxAcceleration = 1.0;
  private static double kP = 0.001;
  private static double kI = 0.0;// keep as 0 because it is not very useful for robotics
  private static double kD = 0.1;
  private static double kS = 1.1;
  private static double kG = 1.2;
  private static double kV = 1.3;
  private final DutyCycleEncoder Encoder = new DutyCycleEncoder(0);
  private final Joystick m_joystick = new Joystick(0);
  final TrapezoidProfile.Constraints m_constraints =
    new TrapezoidProfile.Constraints(kMaxVelocity, kMaxAcceleration);
  final ProfiledPIDController leftclimbController =
    new ProfiledPIDController(kP, kI, kD, m_constraints);
  final ElevatorFeedforward feedforward = new ElevatorFeedforward(kS, kG, kV);
//  private final RelativeEncoder MotorEncoder = leftclimber.getEncoder();

  public ClimberSubsystem() {
    
    this.leftclimber = new CANSparkMax(88, MotorType.kBrushless);//
   // this.rightclimber = new CANSparkMax(0, MotorType.kBrushless);//
   
   
  }
   
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
     if (controller5.button(Constants.buttonList.x) != null) {
      leftclimbController.setGoal(1);
    } else if (m_joystick.getRawButtonPressed(3)) {
      leftclimbController.setGoal(0);
    }
  double encoderValue = Encoder.getAbsolutePosition();
  SmartDashboard.putNumber("encoder", encoderValue);
  
  }
  
public void setClimber(double velocity) {
 //leftclimber.set(speed); 
 //rightclimber.set(-speed);
// final TrapezoidProfile.Constraints m_constraints =
//    new TrapezoidProfile.Constraints(kMaxVelocity, kMaxAcceleration);
//  leftclimber.set(leftclimbController.calculate(Encoder.getAbsolutePosition()));
//+ feedforward.calculate(leftclimbController.getSetpoint().velocity));
//double PIDValue = leftclimbController.calculate(MotorEncoder.getVelocity(),speed);
// leftclimber.set(PIDValue);
}

public void stop() {
 leftclimber.set(0);
 //rightclimber.set(0);
    // TODO Auto-generated method stub
}
}
