// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants;
import frc.robot.Constants.Swerve;
import frc.robot.subsystems.LimelightHelpers;
import frc.robot.subsystems.LimelightHelpers.LimelightResults;
import frc.robot.subsystems.LimelightHelpers.LimelightTarget_Fiducial;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.LimelightHelpers.LimelightTarget_Fiducial;

public class VisionSubsystem extends SubsystemBase {
  /** Creates a new VisionSubsystem. */

    public double AimkP = .035;
    public double AimkI = 0.0;
    public double AimkD = 0.0007;

    public double dkP = 0.1;
    public double dkI = 0.0;
    public double dkD = 0.0001;

    public PIDController AimvisionPID;
    public PIDController distancePID;

    public CANSparkMax testMotor;

    public  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

   public LimelightResults limelight = LimelightHelpers.getLatestResults("limelight");

  public VisionSubsystem() {
    //tx is horizontal value of target, ty is vertical value of target, ta is area, ts is skew of target, tv is validity of target

    LimelightHelpers.setPipelineIndex("limelight",0);
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");
    NetworkTableEntry targetpose_cameraspace = table.getEntry("targetpose_cameraspace");

    this.testMotor = new CANSparkMax(22, MotorType.kBrushless); //7
    
    //read values periodically
    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);
    
    //post to smart dashboard periodically
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);

    // Initializing the angle motor PID Controller with PID values
    this.AimvisionPID = new PIDController(
      AimkP,
      AimkI,
      AimkD
    );

    this.distancePID = new PIDController(
      dkP,
      dkI,
      dkD
    );

  }

  double limelight_aim_proportional()
  {    
    // kP (constant of proportionality)
    // this is a hand-tuned number that determines the aggressiveness of our proportional control loop
    // if it is too high, the robot will oscillate around.
    // if it is too low, the robot will never reach its target
    // if the robot never turns in the correct direction, kP should be inverted.

    // tx ranges from (-hfov/2) to (hfov/2) in degrees. If your target is on the rightmost edge of 
    // your limelight 3 feed, tx should return roughly 31 degrees.
    double targetingAngularVelocity = LimelightHelpers.getTX("limelight");

    // convert to radians per second for our drive method
    targetingAngularVelocity *= Constants.Swerve.maxAngularVelocity;

    //invert since tx is positive when the target is to the right of the crosshair
    targetingAngularVelocity *= -1.0;

    return targetingAngularVelocity;
  }

public double proportionalAiming()
  {    
    // kP (constant of proportionality)
    // this is a hand-tuned number that determines the aggressiveness of our proportional control loop
    // if it is too high, the robot will oscillate around.
    // if it is too low, the robot will never reach its target
    // if the robot never turns in the correct direction, kP should be inverted.

    // tx ranges from (-hfov/2) to (hfov/2) in degrees. If your target is on the rightmost edge of 
    // your limelight 3 feed, tx should return roughly 31 degrees.
    double targetingAngularVelocity = LimelightHelpers.getTX("limelight");
    //double targetingAngularVelocity = table.getEntry("tx").getDouble(0);
    SmartDashboard.putNumber("TX", LimelightHelpers.getTX("limelight"));

    targetingAngularVelocity = AimvisionPID.calculate(targetingAngularVelocity);
    //targetingAngularVelocity = targetingAngularVelocity * AimkP;

    // convert to radians per second for our drive method
    targetingAngularVelocity *= Constants.Swerve.maxAngularVelocity;

    //invert since tx is positive when the target is to the right of the crosshair
    targetingAngularVelocity *= -1.0;



    return targetingAngularVelocity;
  }

  public void drive(double value){

  testMotor.set(value/5);
  System.out.print(table.getEntry("tx").getDouble(0));
  }

  // simple proportional ranging control with Limelight's "ty" value
  // this works best if your Limelight's mount height and target mount height are different.
  // if your limelight and target are mounted at the same or similar heights, use "ta" (area) for target ranging rather than "ty"
  public double getLimelightSpeed()
  {    
    double kP = .1;
    double targetingForwardSpeed = LimelightHelpers.getTY("limelight") * kP;
    targetingForwardSpeed *= -1.0;
    return targetingForwardSpeed;
  }

  public double getDistanceToSpeaker(){
    double ty = LimelightHelpers.getTY("limelight");
    double targetOffsetAngle_Vertical = ty;//not accurate, this is a1, to be changed later

    double desiredDistanceInches = 43.5;//the subwoofer + 6 inches 

    // how many degrees back is your limelight rotated from perfectly vertical?
    double limelightMountAngleDegrees = 28.8; //not accurate to be changed later

    // distance from the center of the Limelight lens to the floor
    double limelightLensHeightInches = 16.875; //not accurate to be changed later

    // distance from the target to the floor
    double goalHeightInches = 57.125; //accounts for the lowest edge

    double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
    double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

    //calculate distance
    double distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);
    double distance = distancePID.calculate(distanceFromLimelightToGoalInches, desiredDistanceInches);
    return distance;
  }

  public double getTXSwerve() {
    return LimelightHelpers.getTX("limelight");
  }

  public boolean apriltagIdCHeck(int ID){

    if (LimelightHelpers.getFiducialID("limelight") == ID){
      return true;
    }
    else {
      return false;
    }

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("SOOOOP", limelight.targetingResults.latency_capture);
  }
}
