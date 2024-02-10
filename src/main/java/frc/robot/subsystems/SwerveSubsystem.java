// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Swerve;
import frc.robot.subsystems.SwerveModule;

public class SwerveSubsystem extends SubsystemBase {
  private final AHRS gyro;
  private final AHRS accelerometer;
  public double angle = 0;

  private SwerveDriveOdometry swerveOdometry;
  private SwerveModule[] mSwerveMods;

  private Field2d field;

  public AutoBuilder autoBuilder;

  private KalmanFilter filter;

  private AHRS ahrs;

  /** Creates a new SwerveSubsystem. */
  public SwerveSubsystem() {
    gyro = new AHRS(SerialPort.Port.kUSB);
    accelerometer = new AHRS(SerialPort.Port.kUSB);

    filter = new KalmanFilter<>(null, null, null, null, null, 0);

    zeroGryo();

    mSwerveMods = new SwerveModule[] {
      new SwerveModule(0, Swerve.Mod0.constants),//test to see if its modul number that denotes position, reguardless of position 
      new SwerveModule(2, Swerve.Mod2.constants),
      new SwerveModule(3, Swerve.Mod3.constants),
      new SwerveModule(1, Swerve.Mod1.constants),
    };

    resetToAbsolute();

    swerveOdometry = new SwerveDriveOdometry(
      Swerve.swerveKinematics, 
      getroll(), 
      getModulePositions()
    );

    field = new Field2d();

    SmartDashboard.putData("Field", field);

     AutoBuilder.configureHolonomic(
            this::getPose, // Robot pose supplier
            this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                    new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
                    4.5, // Max module speed, in m/s
                    0.4, // Drive base radius in meters. Distance from robot center to furthest module.
                    new ReplanningConfig() // Default path replanning config. See the API for the options here
            ),
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this
     );
  }
      
  

  public void driveRobotRelative(ChassisSpeeds Speeds){
    SwerveModuleState[] states = Swerve.swerveKinematics.toSwerveModuleStates(Speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(states, Swerve.maxSpeed);

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(states[mod.moduleNumber], true);
    }
  }
  public void drive(
    Translation2d translation,
    double rotation,
    boolean fieldRelative,
    boolean isOpenLoop
  ) {
    SwerveModuleState[] swerveModuleStates = Swerve.swerveKinematics.toSwerveModuleStates(
      fieldRelative 
        ? ChassisSpeeds.fromFieldRelativeSpeeds(
          translation.getX(),
          translation.getY(), 
          rotation,
          getroll())
        : new ChassisSpeeds(
          translation.getX(),
          translation.getY(), 
          rotation)
    );

    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Swerve.maxSpeed);

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
    }
  }

  public void zeroGryo() {
    gyro.reset();
  }

  public void getAcceleration(){
    accelerometer.getWorldLinearAccelX();
  }

  public void resetToAbsolute() {
    for (SwerveModule mod : mSwerveMods) {
      mod.resetToAbsolute();
    }
  }

  private Rotation2d getroll() {
    if (Swerve.invertGyro) {
      return Rotation2d.fromDegrees(360 - gyro.getRoll());
    } else {
      return Rotation2d.fromDegrees(gyro.getRoll());
    }
  }

  public Rotation2d filterGyro(){
    angle = (0.97402597402)*(angle + (getroll().getDegrees()*0.0262)) + (0.02597402597)*(accelerometer.getWorldLinearAccelX());
    return Rotation2d.fromDegrees(angle);
  }

  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];

    for (SwerveModule mod : mSwerveMods) {
      positions[mod.moduleNumber] = mod.getPosition();
    }

    return positions;
  }

    public Pose2d getPose() {
    return swerveOdometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
    swerveOdometry.resetPosition(getroll(), getModulePositions(), pose);
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(desiredStates[mod.moduleNumber], false);
    }
  }

public ChassisSpeeds getSpeeds() {
  SwerveModuleState[] states = new SwerveModuleState[4];
  for (int i = 0; i < 4; i++) {
    states[i] = mSwerveMods[i].getState();
  }

      return Swerve.swerveKinematics.toChassisSpeeds(states);
    }
    
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    swerveOdometry.update(getroll(), getModulePositions());
    field.setRobotPose(swerveOdometry.getPoseMeters());

    for (SwerveModule mod : mSwerveMods) {
      SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
      SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Integrated", mod.getState().angle.getDegrees());
      SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
      SmartDashboard.putNumber(("GYRO"), getroll().getDegrees());
      SmartDashboard.putNumber("filterGyro", filterGyro().getDegrees());
    }
  }
}
