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
import frc.robot.Constants.Swerve;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveDrive extends Command {
  private final SwerveSubsystem swerveSubsystem;
  private final DoubleSupplier translationSupplier;
  private final DoubleSupplier strafeSupplier;
  private final DoubleSupplier rotationSupplier;
  private final BooleanSupplier robotCentricSupplier;

  private SlewRateLimiter translationLimiter = new SlewRateLimiter(Swerve.kMaxTranslationAcceleration);
  private SlewRateLimiter strafeLimiter = new SlewRateLimiter(Swerve.kMaxStrafeAcceleration);
  private SlewRateLimiter rotationLimiter = new SlewRateLimiter(Swerve.kMaxRotationAcceleration);

  /** Creates a new SwerveDrive. */
  public SwerveDrive(
    SwerveSubsystem swerveSubsystem,
    DoubleSupplier translationSupplier,
    DoubleSupplier strafeSupplier,
    DoubleSupplier rotationSupplier,
    BooleanSupplier robotCentricSupplier
  ) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.swerveSubsystem = swerveSubsystem;
    addRequirements(swerveSubsystem);

    this.translationSupplier = strafeSupplier;//oh god what have I done
    this.strafeSupplier = translationSupplier;
    this.rotationSupplier = rotationSupplier;
    this.robotCentricSupplier = robotCentricSupplier;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    swerveSubsystem.resetToAbsolute();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    SmartDashboard.putNumber("Translation Supplier", translationSupplier.getAsDouble());
    SmartDashboard.putNumber("Strafe Supplier", strafeSupplier.getAsDouble());
    SmartDashboard.putNumber("Rotation Supplier", rotationSupplier.getAsDouble());

    double translationVal = translationLimiter.calculate(
      MathUtil.applyDeadband(translationSupplier.getAsDouble()/2.2, Swerve.stickDeadband));
    double strafeVal = strafeLimiter.calculate(
      MathUtil.applyDeadband(strafeSupplier.getAsDouble()/2.2, Swerve.stickDeadband));
    double rotationVal = rotationLimiter.calculate(
      MathUtil.applyDeadband(rotationSupplier.getAsDouble()/2.2, Swerve.stickDeadband));

    swerveSubsystem.drive(
      new Translation2d(translationVal, -strafeVal).times(Swerve.maxSpeed), 
      rotationVal * Swerve.maxAngularVelocity, 
      robotCentricSupplier.getAsBoolean(),
      true
    );

    SmartDashboard.putNumber("Translation Val", translationVal);
    SmartDashboard.putNumber("Strafe Val", strafeVal);
    SmartDashboard.putNumber("Rotation Val", rotationVal);
    SmartDashboard.putBoolean("Field centric", robotCentricSupplier.getAsBoolean());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
