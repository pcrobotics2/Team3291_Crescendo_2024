// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.Preferences;


public class PreferencesSubsystem extends SubsystemBase{
  public PreferencesSubsystem(String variableName, double variableValue) {
  }

  /** Creates a new Preferences. */
  public double setPreferences (String variableName, double variableValue) {
    Preferences.initDouble(variableName, variableValue);

    if (Preferences.getDouble("variableName", variableValue) != variableValue) {
      variableValue = Preferences.getDouble("variableName", variableValue);
        } 
    return variableValue;
    
}

}
