package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeMotorSubsystem extends SubsystemBase {

  /*-------------------------------- public instance variables ---------------------------------*/
  //public static IntakeMotorSubsystem mInstance;
  //public PeriodicIO mPeriodicIO;

 // public static IntakeMotorSubsystem getInstance() {
 //   if (mInstance == null) {
   //   mInstance = new IntakeMotorSubsystem();
    //}
    //return mInstance;
  //}

  public CANSparkMax IntakeMotorMotor;

  public SparkPIDController IntakeMotorPID;

  public RelativeEncoder IntakeMotorEncoder;

  public SlewRateLimiter mSpeedLimiter = new SlewRateLimiter(1000);

  public IntakeMotorSubsystem() {
    // super("IntakeMotorSubsystem");

  //  mPeriodicIO = new PeriodicIO();

    IntakeMotorMotor = new CANSparkMax(Constants.intake.IntakeID, MotorType.kBrushless);
    IntakeMotorMotor.restoreFactoryDefaults();

    IntakeMotorPID = IntakeMotorMotor.getPIDController();
    IntakeMotorPID.setP(Constants.kLauncherSubP);
    IntakeMotorPID.setI(Constants.kLauncherSubI);
    IntakeMotorPID.setD(Constants.kLauncherSubD);
    IntakeMotorPID.setFF(Constants.kLauncherSubFF);
    IntakeMotorPID.setOutputRange(Constants.kLauncherSubMinOutput, Constants.kLauncherSubMaxOutput);

    mRightLauncherSubPID = mRightLauncherSubMotor.getPIDController();
    mRightLauncherSubPID.setP(Constants.kLauncherSubP);
    mRightLauncherSubPID.setI(Constants.kLauncherSubI);
    mRightLauncherSubPID.setD(Constants.kLauncherSubD);
    mRightLauncherSubPID.setFF(Constants.kLauncherSubFF);
    mRightLauncherSubPID.setOutputRange(Constants.kLauncherSubMinOutput, Constants.kLauncherSubMaxOutput);

    IntakeMotorEncoder = IntakeMotorMotor.getEncoder();
    mRightIntakeMotorSubsystemEncoder = mRightIntakeMotorSubsystemMotor.getEncoder();

    IntakeMotorMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
    mRightIntakeMotorSubsystemMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);

    IntakeMotorMotor.setInverted(true);
    mRightIntakeMotorSubsystemMotor.setInverted(false);

  }

//  public static class PeriodicIO {
//    double IntakeMotorSubsystem_rpm = 0.0;
//  }

  /*-------------------------------- Generic Subsystem Functions --------------------------------*/

  public void stop() {
    stopIntakeMotorSubsystem();
  }

  // public void outputTelemetry() {
  //   putNumber("Speed (RPM):", IntakeMotorSubsystem_rpm);
  //   putNumber("Left speed:", IntakeMotorEncoder.getVelocity());
  //   putNumber("Right speed:", mRightIntakeMotorSubsystemEncoder.getVelocity());
  // }



  /*---------------------------------- Custom Public Functions ----------------------------------*/

  public void setSpeed(double rpm) {
    IntakeMotorMotor.setInverted(true);
    mRightIntakeMotorSubsystemMotor.setInverted(false);
    double limitedSpeed = mSpeedLimiter.calculate(rpm);
    IntakeMotorPID.setReference(limitedSpeed, ControlType.kVelocity);
    mRightIntakeMotorSubsystemPID.setReference(limitedSpeed, ControlType.kVelocity);
  }
  public void setSpeedOpposite(double rpm) {
    IntakeMotorMotor.setInverted(false);
    mRightIntakeMotorSubsystemMotor.setInverted(true);
    double limitedSpeed = mSpeedLimiter.calculate(rpm);
    IntakeMotorPID.setReference(limitedSpeed, ControlType.kVelocity);
    mRightIntakeMotorSubsystemPID.setReference(limitedSpeed, ControlType.kVelocity);
  }

  public void stopIntakeMotorSubsystem() {
    //double limitedSpeed = mSpeedLimiter.calculate(0);
    IntakeMotorPID.setReference(0, ControlType.kVelocity);
    mRightIntakeMotorSubsystemPID.setReference(0, ControlType.kVelocity);
  }

  /*---------------------------------- Custom public Functions ---------------------------------*/
}


