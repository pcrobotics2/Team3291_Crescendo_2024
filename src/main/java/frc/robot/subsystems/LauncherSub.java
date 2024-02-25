package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LauncherSub extends SubsystemBase {

  /*-------------------------------- public instance variables ---------------------------------*/
  //public static LauncherSub mInstance;
  //public PeriodicIO mPeriodicIO;

 // public static LauncherSub getInstance() {
 //   if (mInstance == null) {
   //   mInstance = new LauncherSub();
    //}
    //return mInstance;
  //}

  public CANSparkMax mLeftLauncherSubMotor;
  public CANSparkMax mRightLauncherSubMotor;

  public SparkPIDController mLeftLauncherSubPID;
  public SparkPIDController mRightLauncherSubPID;

  public RelativeEncoder mLeftLauncherSubEncoder;
  public RelativeEncoder mRightLauncherSubEncoder;

  public SlewRateLimiter mSpeedLimiter = new SlewRateLimiter(1000);
  public LauncherSub() {
    // super("LauncherSub");

  //  mPeriodicIO = new PeriodicIO();

    mLeftLauncherSubMotor = new CANSparkMax(Constants.kLauncherSubLeftMotorId, MotorType.kBrushless);
    mRightLauncherSubMotor = new CANSparkMax(Constants.kLauncherSubRightMotorId, MotorType.kBrushless);
   // mLeftLauncherSubMotor.restoreFactoryDefaults();
   //mRightLauncherSubMotor.restoreFactoryDefaults();

    mLeftLauncherSubPID = mLeftLauncherSubMotor.getPIDController();
    mLeftLauncherSubPID.setP(Constants.kLauncherSubP);
    mLeftLauncherSubPID.setI(Constants.kLauncherSubI);
    mLeftLauncherSubPID.setD(Constants.kLauncherSubD);
    mLeftLauncherSubPID.setFF(Constants.kLauncherSubFF);
    mLeftLauncherSubPID.setOutputRange(Constants.kLauncherSubMinOutput, Constants.kLauncherSubMaxOutput);

    mRightLauncherSubPID = mRightLauncherSubMotor.getPIDController();
    mRightLauncherSubPID.setP(Constants.kLauncherSubP);
    mRightLauncherSubPID.setI(Constants.kLauncherSubI);
    mRightLauncherSubPID.setD(Constants.kLauncherSubD);
    mRightLauncherSubPID.setFF(Constants.kLauncherSubFF);
    mRightLauncherSubPID.setOutputRange(Constants.kLauncherSubMinOutput, Constants.kLauncherSubMaxOutput);

    mLeftLauncherSubEncoder = mLeftLauncherSubMotor.getEncoder();
    mRightLauncherSubEncoder = mRightLauncherSubMotor.getEncoder();

    mLeftLauncherSubMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
    mRightLauncherSubMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);

    mLeftLauncherSubMotor.setInverted(true);
    mRightLauncherSubMotor.setInverted(false);

  }

//  public static class PeriodicIO {
//    double LauncherSub_rpm = 0.0;
//  }

  /*-------------------------------- Generic Subsystem Functions --------------------------------*/

  public void stop() {
    stopLauncherSub();
  }

  // public void outputTelemetry() {
  //   putNumber("Speed (RPM):", LauncherSub_rpm);
  //   putNumber("Left speed:", mLeftLauncherSubEncoder.getVelocity());
  //   putNumber("Right speed:", mRightLauncherSubEncoder.getVelocity());
  // }



  /*---------------------------------- Custom Public Functions ----------------------------------*/

  public void setSpeed(double rpm) {
    mLeftLauncherSubMotor.setInverted(true);
    mRightLauncherSubMotor.setInverted(false);
    double limitedSpeed = mSpeedLimiter.calculate(rpm);
    mLeftLauncherSubPID.setReference(limitedSpeed, ControlType.kVelocity);
    mRightLauncherSubPID.setReference(limitedSpeed, ControlType.kVelocity);
  }
  public void setSpeedOpposite(double rpm) {
    mLeftLauncherSubMotor.setInverted(false);
    mRightLauncherSubMotor.setInverted(true);
    double limitedSpeed = mSpeedLimiter.calculate(rpm);
    mLeftLauncherSubPID.setReference(limitedSpeed, ControlType.kVelocity);
    mRightLauncherSubPID.setReference(limitedSpeed, ControlType.kVelocity);
  }

  public void stopLauncherSub() {
    //double limitedSpeed = mSpeedLimiter.calculate(0);
    mLeftLauncherSubPID.setReference(0, ControlType.kVelocity);
    mRightLauncherSubPID.setReference(0, ControlType.kVelocity);
  }

  /*---------------------------------- Custom public Functions ---------------------------------*/
}


