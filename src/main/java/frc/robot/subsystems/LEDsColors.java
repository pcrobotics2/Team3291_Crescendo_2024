// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.subsystems.ColorChanger;

public class LEDsColors extends SubsystemBase {
  private static LEDsColors m_instance;


  private int m_ledTotalLength = Constants.LEDColors.k_totalLength;
  
  //private Function<Integer, Function<Integer>LEDsColors;

  public static LEDsColors getInstanceColors() {
    if (m_instance == null) {
      m_instance = new LEDsColors();

    }
    return m_instance;
  }
  private LEDsColors() {
    super("LEDsColors");
    m_LEDsColors = new LEDsColors();
    m_LEDsColors.setLength(m_ledTotalLength);
    m_LEDsColors.start();
  }

  @Override
  public void periodic() {
    setColorMode();
  }

  public void setColor(Color color) {
    m_ledStripColor = LEDsColors.setColor(color);
  }

  public void defaultLEDsColors() {
    
  }
}
