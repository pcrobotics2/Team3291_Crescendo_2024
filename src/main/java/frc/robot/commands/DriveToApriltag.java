// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.Swerve;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class DriveToApriltag extends Command {
  private final VisionSubsystem visionSubsystem;
  private final SwerveSubsystem swerveSubsystem;
  private double rotationVal;
  private double translationVal;
  private double strafeVal;
  private double visionTX;

  //MAYBE USE TX FOR STRAFING AND THEN SKEW FOR ROT, TEST THIS FORST BUT DEFINTELY CONSIDER THIS AS A POSIBILITY

  private SlewRateLimiter translationLimiter = new SlewRateLimiter(Swerve.kMaxTranslationAcceleration);
  //private SlewRateLimiter strafeLimiter = new SlewRateLimiter(Swerve.kMaxStrafeAcceleration);
  private SlewRateLimiter rotationLimiter = new SlewRateLimiter(Swerve.kMaxRotationAcceleration);
  private int apriltagID;

  /** Creates a new SwerveDrive. */
  public DriveToApriltag(
    SwerveSubsystem swerveSubsystem,
    VisionSubsystem visionSubsystem,
    int apriltagID
  ) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.visionSubsystem = visionSubsystem;
    this.swerveSubsystem = swerveSubsystem;
    this.apriltagID = apriltagID;
    addRequirements(swerveSubsystem, visionSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    swerveSubsystem.resetToAbsolute();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (visionSubsystem.apriltagIdCHeck(apriltagID) == true && apriltagID != 0){

      if (visionSubsystem.getTXSwerve() > 1 || visionSubsystem.getTXSwerve() < -1) {
      if (visionSubsystem.getTXSwerve() > 20) {
        this.visionTX = 20;
      }
      if (visionSubsystem.getTXSwerve() < -20) {
        this.visionTX = -20;
      }
      else {
        this.visionTX = visionSubsystem.getTXSwerve();
      }
    }
      else {
        this.visionTX = Constants.Swerve.visionXOffset;
      }

      double visionOutput = (visionTX - Constants.Swerve.visionXOffset)/20;
      this.rotationVal = rotationLimiter.calculate(visionOutput);

      this.translationVal = translationLimiter.calculate(visionSubsystem.getDistance());
      this.strafeVal = 0.0;

    swerveSubsystem.drive(
      new Translation2d(this.translationVal, this.strafeVal).times(Swerve.maxSpeed), 
      this.rotationVal * Swerve.maxAngularVelocity, 
      false,
      false
    );

  } else {
    swerveSubsystem.drive(
      new Translation2d(0.0, 0.0).times(Swerve.maxSpeed), 
      0.1 * Swerve.maxAngularVelocity, 
      false,
      false
    );
  }

    SmartDashboard.putNumber("Translation Val", this.translationVal);
    SmartDashboard.putNumber("Strafe Val", this.strafeVal);
    SmartDashboard.putNumber("Rotation Val", this.rotationVal);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    if (rotationVal == 0 && Math.abs(this.translationVal) >= 0.01){
      return true;
    }
    return false;
  }
}
