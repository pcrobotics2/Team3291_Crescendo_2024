// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.config.SwerveModuleConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
  
  public static final int leftClimberID = 39;
  public static final int rightClimberID = 22;

  public static final double feedSpeed = 0.40;   //0.40
  public static final double launchSpeed = 0.90; //1.00 originally
  public static final double timeToWait = 0.25;
  public static final double hangSpeed = 0.5;
  public static final int upLauncherID = 18; //18
  public static final int downLauncherID = 14; //14

  public static class buttonList {
    public static final int a = 1;
    public static final int b = 2; 
    public static final int x = 3; 
    public static final int y = 4; 
    public static final int lb = 5; 
    public static final int rb = 6; 
    public static final int back = 7; 
    public static final int start = 8; 
    public static final int l3 = 9; 
    public static final int r3 = 10; 

  }
  public static class intake {

    public static class intakePID {
    public static final double kp = 0.1;
    public static final double ki = 0.01;
    public static final double kd = 0.0;
    }

    //ids
    public static final int encoderID = 0; //Changed due to rewire
    public static final int IntakeID = 19; //21
    public static final int PivotID = 25;//19 
    public static final int intakeLimitSwitchID = 9;

    public static final double k_pivotEncoderOffset = 40;

    //angles
    public static final double groundAngle = 10;//doesn't like 0
    public static final double stowAngle = 163;
    public static final double sourceAngle = 95;
    public static final double ampAngle = 70;

    public static final double ejectSpeed = 0.5;
    public static final double intakeSpeed = 0.7;
    
  }


  public static class Swerve {
    public static final double stickDeadband = 0.1;//configure and mess around with later
    public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW- 

    public static final double trackWidth = Units.inchesToMeters(21.25);
    public static final double wheelBase = Units.inchesToMeters(21.25);
    public static final double wheelDiameter = Units.inchesToMeters(4.0);
    public static final double wheelCircumference = wheelDiameter * Math.PI;
    public static final double driveGearRatio = (6.75 / 1.0); // 6.75:1
    public static final double angleGearRatio = ((150/7) / 1.0); // 150/7:1

    public static final double kMaxTranslationAcceleration = 3.0; //meters per second squared
    public static final double kMaxStrafeAcceleration = 3.0; //meters per second squared
    public static final double kMaxRotationAcceleration = 3.0; //radians per second squared

    // inematics
    public static final SwerveDriveKinematics swerveKinematics =
        new SwerveDriveKinematics(
            //front left ++ 
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            //front right +-
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            //back left -+
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            //Back right --
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

    /* Swerve Profiling Values */
    // Meters per second IN THEORY
    public static final double maxSpeed = 4.4196;

    /**
     * Find out?
     * 
     * In theory the max speed is already given, but I can't find the max
     * angular velocity, so we'll probably calculate both anyways
     */
    public static final double maxAngularVelocity = 11.5;

    // Swerve Voltage Compensation
    public static final double voltageComp = 12.0;

    /* Drive Motor Conversion Factors */
    // The conversion factor to multiply the native units by 
    public static final double driveConversionPositionFactor =
        (wheelDiameter * Math.PI) / driveGearRatio;

    // The conversion factor to multiply the native units by 
    public static final double driveConversionVelocityFactor = driveConversionPositionFactor / 60.0;

    // The conversion factor to multiply the native units by (angle position)
    public static final double angleConversionFactor = 360.0 / angleGearRatio;

    // Swerve Current Limiting
    public static final int angleContinuousCurrentLimit = 20;
    public static final int driveContinuousCurrentLimit = 80;

    // Motor Inverts
    public static final boolean driveInvert = false;
    public static final boolean angleInvert = true;

    // Angle Encoder Invert
    public static final boolean canCoderInvert = false;

    // Angle Motor PID Values
    public static final double angleKP = 0.003; //play around with/tune this later 
    public static final double angleKI = 0.0000001;
    public static final double angleKD = 0.0001;
    public static final double angleKFF = 0.0;

    // Drive Motor PID Values
    public static final double driveKP = 0.0001;
    public static final double driveKI = 0.0;
    public static final double driveKD = 0.0;

    //feedforward constants
    public static final double ffkS = 0.667;
    public static final double ffkV = 2.44;
    public static final double ffkA = 0.27;

    /**************************
     * Module Specific Constants
     **************************/
    // Front Left Module - Module 0 
    public static final class Mod0 {
      // SparkMAX CAN Device ID
      public static final int driveMotorID = 10;

      // SparkMAX CAN Device ID
      public static final int angleMotorID = 11;

      // CANCoder CAN Device ID
      public static final int canCoderID = 2;

      // Wheel starting angle offset
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(325.019531250);

      // Constants in a nice package.
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    // Back Right Module - Module 1 //then left back
    public static final class Mod1 {
      // SparkMAX CAN Device ID
      public static final int driveMotorID = 6;

      // SparkMAX CAN Device ID
      public static final int angleMotorID = 5;

      // CANCoder CAN Device ID
      public static final int canCoderID = 3;

      // Wheel starting angle offset
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(147.041015625);

      // Constants in a nice package.
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    // Back Left Module - Module 2
    public static final class Mod2 {
      // SparkMAX CAN Device ID
      public static final int driveMotorID = 12;

      // SparkMAX CAN Device ID
      public static final int angleMotorID = 13; //13

      // CANCoder CAN Device ID
      public static final int canCoderID = 1;

      // Wheel starting angle offset
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(174.8144);

      // Constants in a nice package.
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    // Front Right Module - Module 3
    public static final class Mod3 {
      // SparkMAX CAN Device ID
      public static final int driveMotorID = 9;

      // SparkMAX CAN Device ID
      public static final int angleMotorID = 8;

      // CANCoder CAN Device ID
      public static final int canCoderID = 0;

      // Wheel starting angle offset
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(284.326171875);

      // Constants in a nice package.
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }
  }

    public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 4.4196; 
    public static final double kMaxAccelerationMetersPerSecondSquared = 5;
    public static final double kMaxAngularSpeedRadiansPerSecond = 11.5;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = 11.5 * 11.5;
    public static final double kPXController = 0.5;
    public static final double kPYController = 0.5;
    public static final double kPThetaController = 0.5;

    // Constraint for the motion profilied robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public final static class Lighting {
    public final static int lightingPort = 1;

    public enum Colors {
     RAINBOWRAINBOW  ("Rainbow Palette", -0.99),
     RAINBOWPARTY ("Rainbow Party Palette", -0.97),
     RAINBOWOCEAN ("Rainbow Ocean Palette", -0.95),
     RAINBOWLAVE ("Rainbow Lave Palette", -0.93),
     RAINBOWFOREST ("Rainbow Forest Palette", -0.91),
     RAINBOWGLITTER ("Rainbow with Glitter", -0.89),
     CONFETTI ("Confetti", -0.87),
     SHOTRED ("Shot Red", -0.85),
     SHOTBLUE ("Shot Blue", -0.83),
     SHOTWHITE ("Shot White", -0.81),
     SINELONRAINBOW ("Sinelon Rainbow Palette", -0.79),
     SINELONPARTY ("Sinelon Party Palette", -0.77),
     SINELONOCEAN ("Sinelon Ocean Palette", -0.75),
     SINELONLAVA ("Sinelon Lava Palette", -0.73),
     SINELONFOREST ("Sinelon Forest Palette", -0.71),
     BEATSRAINBOWPALETTE ("Beats per Minute Rainbow Palette", -0.69),
     BEATSPARTYPALETTE ("Beats per Minute Party Pallette", -0.67),
     BEATSOCEANPALETTE ("Beats per Minute Ocean Pallette", -0.65),
     BEATSLAVAPALETTE ("Beats per Minute Lava Pallette", -0.63),
     BEATSFORESTPALETTE ("Beats per Minute Forest Pallette", -0.61),
     FIREMEDIUM ("Fire Medium", -0.59),
     FIRELARGE ("Fire Large", -0.57),
     TWINKLESRAINBOW ("Twinkles Rainbow Palette", -0.55),
     TWINKLESPARTY ("Twinkles Party Palette", -0.53),
     TWINKLESOCEAN ("Twinkles Ocean Palette", -0.51),
     TWINKLESLAVA ("Twinkles Lava Palette", -0.49),
     TWINKLESFOREST ("Twinkles Forest Palette", -0.47),
     COLORWAVESRAINBOW ("Color Waves Rainbow Palette", -0.45),
     COLORWAVESPARTY ("Color Waves Party Palette", -0.43),
     COLORWAVESOCEAN ("Color Waves Ocean Palette", -0.41),
     COLORWAVESLAVA ("Color Waves Lava Palette", -0.39),
     COLORWAVESFOREST ("Color Waves Forest Palette", -0.37),
     LARSONRED ("Larson Scanner Red", -0.35),
     LARSONGRAY ("Larson Scanner Gray", -0.33),
     CHASERED ("Light Chase Red", -0.31),
     CHASEBLUE ("Light Chase Blue", -0.29),
     CHASEGRAY ("Light Chase Gray", -0.27),
     HEARTBEATRED ("Heartbeat Red", -0.25),
     HEARTBEATBLUE ("Heartbeat Blue", -0.23),
     HEARTBEATWHITE ("Heartbeat White", -0.21),
     HEARTBEATGRAY ("Heartbeat Gray", -0.19),
     BREATHRED ("Breath Red", -0.17),
     BREATHBLUE ("Breath Blue", -0.15),
     BREATHGRAY ("Breath Gray", -0.13),
     STROBERED ("Strobe Red", -0.11),
     STROBEBLUE ("Strobe Blue", -0.09),
     STROBEGOLD ("Strobe Gold", -0.07),
     STROBEWHITE ("Strobe White", -0.05),
     ENDTOOFF ("End to End Blend to Off", -0.03),
     LARSONSCANNER ("Larson Scanner", -0.01),
     LIGHTCHASE ("Light Chase", 0.01),
     HEARTBEATSLOW ("Heartbeat Slow", 0.03),
     HEARTBEATMEDIUM ("Heartbeat Medium", 0.05),
     HEARTBEATFAST ("Heartbeat Fast", 0.07),
     BREATHSLOW ("Breath Slow", 0.09),
     BREATHFAST ("Breath Fast", 0.11),
     SHOT ("Shot", 0.13),
     STROBE ("Strobe", 0.15),
     ENDTOOFFTWO ("End to End Blend to Off Two", 0.17),
     LARSONSCANNERTWO ("Larson Scanner Two", 0.19),
     LIGHTCHASETWO ("Light Chase Two", 0.21),
     HEARTBEATSLOWTWO ("Heartbeat Slow Two", 0.23),
     HEARTBEATMEDIUMTWO ("Heartbeat Medium Two", 0.25),
     HEARTBEATFASTTWO ("Heartbeat Fast Two", 0.27),
     BREATHSLOWYTWO ("Breath Slow Two", 0.29),
     BREATHFASTTWO ("Breath Slow Two", 0.31),
     SHOTTWO ("Shot Two", 0.33),
     STROBETWO ("Strobe Two", 0.35),
     SPARKLEONEANDTWO ("Sparkle Color One on Color Two", 0.37),
     SPARKLETWOANDONE ("Sparkle Color Two on Color One", 0.39),
     COLORGRADIENTONEANDTWO ("Color Gradient Color One and Two", 0.41),
     BEATSCOLORONEANDTWO ("Beats per Minute Color One and Two", 0.43),
     ENDTOENDOFF ("End to End Blend Color One and Two", 0.45),
     ENDTOEND ("End to End Blend", 0.47),
     COLORONEANDCOLORTWO ("Color Ome and Color Two no blending", 0.49),
     TWINKLESCOLORS ("Twinkles Color One and Two", 0.51),
     COLORWAVESONEANDTWO ("Color Waves Color One and Two", 0.53),
     SINELONONEANDTWO ("Sinelon Color One and Two", 0.55),
     HOTPINK ("Hot Pink", 0.57),
     DARKRED ("Dark Red", 0.59),
     RED ("Red", 0.61),
     REDORANGE ("Red Orange", 0.63),
     ORANGE ("Orange", 0.65),
     GOLD ("Gold", 0.67),
     YELLOW ("Yellow", 0.69),
     LAWNGREEN ("Lawn Green", 0.71),
     LIME ("Lime", 0.73),
     DARKGREEN ("Dark Green", 0.75),
     GREEN ("Green", 0.77),
     BLUEGREEN ("Blue Green", 0.79),
     AQUA ("Auqa", 0.81),
     SKYBLUE ("Sky Blue", 0.83),
     DARKBLUE ("Dark Blue", 0.85),
     BLUE ("Blue", 0.87),
     BLUEVIOLET ("Blue Violet", 0.89),
     VIOLET ("Violet", 0.91),
     WHITE ("White", 0.93),
     GRAY ("Gray", 0.95),
     DARKGRAY ("Dark Gray", 0.97),
     OFF ("Off", 0.99);

     public final String colorName;
     public final double colorValue;

     Colors(String colorName, double colorValue) {
      this.colorName = colorName;
      this.colorValue = colorValue;
    }

    public String getColorName() {
      return this.colorName;
    }

    public double getColorValue() {
      return this.colorValue;
     }
    }

    public final static Colors startingColor = Colors.OFF;
    public final static Colors disableColor = Colors.RAINBOWOCEAN;
  }
}