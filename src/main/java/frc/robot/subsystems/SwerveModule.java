package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.config.CTREConfigs;
import frc.lib.config.SwerveModuleConstants;
import frc.lib.util.CANCoderUtil;
import frc.lib.util.CANCoderUtil.CCUsage;
import frc.lib.util.CANSparkMaxUtil;
import frc.lib.util.CANSparkMaxUtil.Usage;
import frc.robot.Constants;
import frc.robot.Constants.Swerve;
import frc.lib.Math.SwerveOpt;
import frc.lib.config.*;

public class SwerveModule {
    // Module number identifier
    public int moduleNumber;

    // This is the last angle that we wanted to the wheel to move to.
    private Rotation2d lastAngle;

    /**
     * This is the starting angle of the wheel.   Since motors can be motor 
     * differently and the starting position of the shaft starts in a different
     * location, we need to know what the starting angle when the wheels are 
     * facing forward.   This is the basis of all angle changes after the 
     * robot is started up.
     */
    private Rotation2d angleOffset;

    // Used to angle the wheel in the directions we want to drive.
    private CANSparkMax angleMotor;

    // Used to rotate the wheel to move the robot.
    private CANSparkMax driveMotor;

    // Used to get the current position and state (velocity) of the drive motor
    private RelativeEncoder driveEncoder;

    /**
     * Currently defined, but not used in any calculations.   I'm assuming this was
     * first defined to be used for the angle motor, but we are doing that with the 
     * CANCoder (angleEncoder) defined below.
     */
    private RelativeEncoder integratedAngleEncoder;

    // Used to get the current angle (direction) of the wheel 
    private CANCoder angleEncoder;

    /** 
     * Used to PID control the angle motor power level.   The angle is converted from 
     * degrees to percentage of power.
     */
    private PIDController anglePid;

    /**
     * 
     * @param moduleNumber
     * @param moduleConstants
     */
    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
        // This is the module identifier.
        this.moduleNumber = moduleNumber;

        // This is the offset of the wheel in relation to the 0 position of the CANCoder.
        this.angleOffset  = moduleConstants.angleOffset;

        // Initializing the angle motor PID Controller with PID values
        this.anglePid = new PIDController(
            Swerve.angleKP,
            Swerve.angleKI,
            Swerve.angleKD
        );

        //this.anglePid.enableContinuousInput(-Math.PI, Math.PI);

        // Initializing the CANCoder with the desired device ID
        this.angleEncoder = new CANCoder(moduleConstants.canCoderId);//CANcoder(moduleConstants.canCoderId);
        configAngleEncoder();

        //configuring and initalizing drive motors and encoders
        this.driveMotor = new CANSparkMax(moduleConstants.driveMotorId, MotorType.kBrushless);
        this.driveEncoder = driveMotor.getEncoder();
        // this.driveController = driveMotor.getPIDController();
        this.configDriveMotor();

        //configuring and initalizing angle motors and encoders
        this.angleMotor = new CANSparkMax(moduleConstants.angleMotorId, MotorType.kBrushless);
        this.integratedAngleEncoder = angleMotor.getEncoder();
        // this.angleController = angleMotor.getPIDController();
        this.configAngleMotor();
    
        this.lastAngle = getState().angle;
    }
    
    /* 
     * 
     * This is intended to set the default strategy for each angle encoder (CANCoder)
     */
    private void configAngleEncoder() {
        angleEncoder.configFactoryDefault();
        CANCoderUtil.setCANCoderBusUsage(angleEncoder, CCUsage.kMinimal);
        angleEncoder.configAllSettings(CTREConfigs.swerveCanCoderConfig);//Settings is null
    }

    /**
     * configDriveMotor() 
     * 
     * This is intended to set default values for this modules drive motor (SparkMAX)
     */
    private void configDriveMotor() {
        // Set motor to original specs
        this.driveMotor.restoreFactoryDefaults();

        // This sets the transmission rate of data all channels to be faster
        CANSparkMaxUtil.setCANSparkMaxBusUsage(driveMotor, Usage.kAll);

        // Sets the current limit to the drive motor in AMPs
        this.driveMotor.setSmartCurrentLimit(Swerve.driveContinuousCurrentLimit);

        // Invert drive motor (currently, false)
        this.driveMotor.setInverted(Swerve.driveInvert);

        // When the motor isn't being used, set to brake mode
        this.driveMotor.setIdleMode(IdleMode.kBrake);

        // Set the conversion factor for velocity of the encoder. Multiplied by the native output units to give you velocity
        this.driveEncoder.setVelocityConversionFactor(Swerve.driveConversionVelocityFactor);

        // Set the conversion factor for position of the encoder. Multiplied by the native output units to give you position.
        this.driveEncoder.setPositionConversionFactor(Swerve.driveConversionPositionFactor);

        // Sets the voltage compensation setting for all modes on the SPARK and enables voltage compensation.
        this.driveMotor.enableVoltageCompensation(Swerve.voltageComp);

        // Writes values to the motor firmware
        this.driveMotor.burnFlash();

        // Set starting position of the drive encoder to 0.0
        this.driveEncoder.setPosition(0.0);
    }

    /**
     * 
     */
    private void configAngleMotor() {
        // Set motor to the original specs
        this.angleMotor.restoreFactoryDefaults();

        // Sets the transmission rate of the position channel as faster.
        CANSparkMaxUtil.setCANSparkMaxBusUsage(angleMotor, Usage.kPositionOnly);

        CANCoderUtil.setCANCoderBusUsage(angleEncoder, CCUsage.kMinimal);

        // Sets the current limit to the angle motor in AMPs
        this.angleMotor.setSmartCurrentLimit(Swerve.angleContinuousCurrentLimit);

        // Invert angle motor (currently, true)
        this.angleMotor.setInverted(Swerve.angleInvert);

        // WHen the motor isn't being used, set to brake mode
        this.angleMotor.setIdleMode(IdleMode.kBrake);

        // Set the conversion factor for position of the encoder. Multiplied by the native output units to give you position.
        this.integratedAngleEncoder.setPositionConversionFactor(Swerve.angleConversionFactor);

        // Sets the voltage compensation setting for all modes on the SPARK and enables voltage compensation.
        this.angleMotor.enableVoltageCompensation(Swerve.voltageComp);

        // Writes values to the motor firmware
        this.angleMotor.burnFlash();
    }

    /**
     * 
     * @param desiredState
     */
    private void setAngle(SwerveModuleState desiredState) {
        Rotation2d desiredAngle;
        
        // Prevent rotating module if speed is less then 1%. Prevents jittering.
        if ((Math.abs(desiredState.speedMetersPerSecond) / Swerve.maxSpeed) < 0.01) {
            desiredAngle = this.lastAngle;
        } else {
            desiredAngle = desiredState.angle;
        }
            
        // Save the last angle we wanted to move too
        this.lastAngle = desiredAngle;
    
        Rotation2d currentAngle = this.getCanCoder();

        // Returns -180 to 180
        Double currentDegrees = currentAngle.getDegrees(); 

        // Returns -180 to 180 plus the angle offset constant we determined for this module
        Double desiredDegrees = desiredAngle.getDegrees() + this.angleOffset.getDegrees(); 
        
        /**
         * Calculate the difference between the current degrees and desired degrees.
         * Adding 180 to convert it back to 360 degress
         * Then convert back to -180 to 180
         * 
         * At least that's what I think it's doing.
         */
         Double diffDegrees = (currentDegrees - desiredDegrees + 180) % 360 - 180;
    
        if (diffDegrees < -180) {
            diffDegrees = diffDegrees + 360;
        }

        // Calculate the PID value of -1 to 1 based on the degrees we calculated above
        
        Double value = this.anglePid.calculate(diff, 0);

        //just in case the pid value goes over the designated limit, bam 
        value = value > 1 ? 1 : value;
        value = value < -1 ? -1 : value;

        SmartDashboard.putNumber("Value", value);
        //System.out.println("Hats");
    
        // Add turn the angular motor.
        this.angleMotor.set(value); 

        //returns the drive motors direction
        return invertDriveMotor;
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop, boolean invertDriveMotor) {
        // If we are in open loop mode, set the drive motor to the desired speed
        // if (isOpenLoop) {
            double driveValue = (desiredState.speedMetersPerSecond / Swerve.maxSpeed);
            this.driveMotor.set(invertDriveMotor ? driveValue * -1 : driveValue);
        // } else {
        //     // If we are in closed loop mode, set the drive motor to the desired speed
        //     this.driveMotor.set(
        //         this.driveEncoder.getVelocityConversionFactor() * 
        //        desiredState.speedMetersPerSecond / Swerve.maxSpeed
        //    );
        // }
    }

    public Rotation2d getCanCoder() {
        return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(this.driveEncoder.getVelocity(), this.getCanCoder());
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            (this.driveEncoder.getPosition() * (Swerve.wheelCircumference / (Swerve.driveGearRatio * 42))), 
            this.getCanCoder()
        );
    }
    
    public void resetToAbsolute() {
      this.lastAngle = Rotation2d.fromDegrees(0);
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        //desiredState = SwerveModuleState.optimize(desiredState, getState().angle);
        setAngle(desiredState);
        setSpeed(desiredState, isOpenLoop);
    }


}