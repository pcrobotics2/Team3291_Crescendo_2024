package frc.lib.util;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;

public class CANSparkMaxUtil {
    public enum Usage {
        kAll,
        kPositionOnly,
        kVelocityOnly,
        kMinimal
    };

    /**
     * This function allows reducing a Spark Max's CAN bus utilization by reducing the periodic status
     * frame period of nonessential frames from 20ms to 500ms.
     *
     * <p>See
     * https://docs.revrobotics.com/sparkmax/operating-modes/control-interfaces#periodic-status-frames
     * for a description of the status frames.
     *
     * @param motor The motor to adjust the status frame periods on.
     * @param usage The status frame feedack to enable. kAll is the default when a CANSparkMax is
     *     constructed.
     * @param enableFollowing Whether to enable motor following.
     */
    public static void setCANSparkMaxBusUsage(
        CANSparkMax motor,          // SparkMAX motor to configure
        Usage usage,                // Enum above, used to determine which config
        boolean enableFollowing     // Used to determine following
    ) {
        // Set rate of transmission for Status0
        if (enableFollowing) {
            motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus0, 10);
        } else {
            motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus0, 500);
        }

        if (usage == Usage.kAll) {
            // Increase rate for all Status channels
            motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus1, 20);
            motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus2, 20);
            motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus3, 50);
        } else if (usage == Usage.kPositionOnly) {
            // Only increase Position (Status2) channel
            motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus1, 500);
            motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus2, 20);
            motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus3, 500);
        } else if (usage == Usage.kVelocityOnly) {
            // Only increase Velocity (Status1) channel
            motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus1, 20);
            motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus2, 500);
            motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus3, 500);
        } else if (usage == Usage.kMinimal) {
            // Setting minimal rate for all channels.
            motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus1, 500);
            motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus2, 500);
            motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus3, 500);
        }
    }

    /**
     * This function allows reducing a Spark Max's CAN bus utilization by reducing the periodic status
     * frame period of nonessential frames from 20ms to 500ms.
     *
     * <p>See
     * https://docs.revrobotics.com/sparkmax/operating-modes/control-interfaces#periodic-status-frames
     * for a description of the status frames.
     *
     * @param motor The motor to adjust the status frame periods on.
     * @param usage The status frame feedack to enable. kAll is the default when a CANSparkMax is
     *     constructed.
     */
    public static void setCANSparkMaxBusUsage(CANSparkMax motor, Usage usage) {
        setCANSparkMaxBusUsage(motor, usage, false);
    }
}
