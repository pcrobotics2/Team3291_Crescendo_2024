// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Lighting;
import frc.robot.Constants.Lighting.Colors;

public class ColorChanger extends SubsystemBase { 
  public Spark lighting;

  public final SendableChooser<Constants.Lighting.Colors> lighting_chooser =  new SendableChooser<>();

  /** Creates a new ColorChanger. */
 public ColorChanger() {
 
    lighting = new Spark(Constants.Lighting.lightingPort);

    lighting.set(Constants.Lighting.startingColor.getColorValue());

    lighting_chooser.setDefaultOption(Constants.Lighting.startingColor.getColorName(), Constants.Lighting.startingColor);

    for (Colors c : Colors.values()) {
      lighting_chooser.addOption(c.getColorName(), c);
    }
    SmartDashboard.putData("Alliance", lighting_chooser);
  }

  public void blink(){
    Colors selectedColor = lighting_chooser.getSelected();

    lighting.set(selectedColor.getColorValue());
    lighting.set(0.99);
  }
 
  public void defaultDisable() {
    lighting.set(Constants.Lighting.disableColor.getColorValue());
  }
  public void setRED() {
  }
  public void setGREEN() {
  }
  public void setGOLD() {
  }
  public void setRAINBOWRAINBOW() {}
  @Override
  public void periodic() {
    //This method will be called once per scheduler run
    Colors selectedColor = lighting_chooser.getSelected();

    lighting.set(selectedColor.getColorValue());
  }

  public static void set(double colorValue) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'set'");
  }
}
