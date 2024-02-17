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


public void setRAINBOWRAINBOW() {
    lighting.set(Constants.Lighting.Colors.ColorConstants.RAINBOWRAINBOW);
}

public void setRAINBOWPARTY() {
    lighting.set(Constants.Lighting.Colors.ColorConstants.RAINBOWPARTY);
}

public void setRAINBOWOCEAN() {
    lighting.set(Constants.Lighting.Colors.ColorConstants.RAINBOWOCEAN);
}

public void setRAINBOWLAVE() {
    lighting.set(Constants.Lighting.Colors.ColorConstants.RAINBOWLAVE);
}

public void setRAINBOWFOREST() {
    lighting.set(Constants.Lighting.Colors.ColorConstants.RAINBOWFOREST);
}

public void setRAINBOWGLITTER() {
    lighting.set(Constants.Lighting.Colors.ColorConstants.RAINBOWGLITTER);
}

public void setCONFETTI() {
    lighting.set(Constants.Lighting.Colors.ColorConstants.CONFETTI);
}

public void setSHOTRED() {
    lighting.set(Constants.Lighting.Colors.ColorConstants.SHOTRED);
}

public void setSHOTBLUE() {
    lighting.set(Constants.Lighting.Colors.ColorConstants.SHOTBLUE);
}

public void setSHOTWHITE() {
    lighting.set(Constants.Lighting.Colors.ColorConstants.SHOTWHITE);
}

public void setSINELONRAINBOW() {
    lighting.set(Constants.Lighting.Colors.ColorConstants.SINELONRAINBOW);
}

public void setSINELONPARTY() {
    lighting.set(Constants.Lighting.Colors.ColorConstants.SINELONPARTY);
}

public void setSINELONOCEAN() {
    lighting.set(Constants.Lighting.Colors.ColorConstants.SINELONOCEAN);
}

public void setSINELONLAVA() {
    lighting.set(Constants.Lighting.Colors.ColorConstants.SINELONLAVA);
}

public void setSINELONFOREST() {
    lighting.set(Constants.Lighting.Colors.ColorConstants.SINELONFOREST);
}

public void setBEATSRAINBOWPALETTE() {
    lighting.set(Constants.Lighting.Colors.ColorConstants.BEATSRAINBOWPALETTE);
}

public void setBEATSPARTYPALETTE() {
    lighting.set(Constants.Lighting.Colors.ColorConstants.BEATSPARTYPALETTE);
}

public void setBEATSOCEANPALETTE() {
    lighting.set(Constants.Lighting.Colors.ColorConstants.BEATSOCEANPALETTE);
}

public void setBEATSLAVAPALETTE() {
    lighting.set(Constants.Lighting.Colors.ColorConstants.BEATSLAVAPALETTE);
}

public void setBEATSFORESTPALETTE() {
    lighting.set(Constants.Lighting.Colors.ColorConstants.BEATSFORESTPALETTE);
}

public void setFIREMEDIUM() {
    lighting.set(Constants.Lighting.Colors.ColorConstants.FIREMEDIUM);
}

public void setFIRELARGE() {
    lighting.set(Constants.Lighting.Colors.ColorConstants.FIRELARGE);
}

public void setTWINKLESRAINBOW() {
    lighting.set(Constants.Lighting.Colors.ColorConstants.TWINKLESRAINBOW);
}

public void setTWINKLESPARTY() {
    lighting.set(Constants.Lighting.Colors.ColorConstants.TWINKLESPARTY);
}

public void setTWINKLESOCEAN() {
    lighting.set(Constants.Lighting.Colors.ColorConstants.TWINKLESOCEAN);
}

public void setTWINKLESLAVA() {
    lighting.set(Constants.Lighting.Colors.ColorConstants.TWINKLESLAVA);
}

public void setTWINKLESFOREST() {
    lighting.set(Constants.Lighting.Colors.ColorConstants.TWINKLESFOREST);
}

public void setCOLORWAVESRAINBOW() {
    lighting.set(Constants.Lighting.Colors.ColorConstants.COLORWAVESRAINBOW);
}

public void setCOLORWAVESPARTY() {
    lighting.set(Constants.Lighting.Colors.ColorConstants.COLORWAVESPARTY);
}

public void setCOLORWAVESOCEAN() {
    lighting.set(Constants.Lighting.Colors.ColorConstants.COLORWAVESOCEAN);
}

public void setCOLORWAVESLAVA() {
    lighting.set(Constants.Lighting.Colors.ColorConstants.COLORWAVESLAVA);
}

public void setCOLORWAVESFOREST() {
    lighting.set(Constants.Lighting.Colors.ColorConstants.COLORWAVESFOREST);
}

public void setLARSONRED() {
    lighting.set(Constants.Lighting.Colors.ColorConstants.LARSONRED);
}

public void setLARSONGRAY() {
    lighting.set(Constants.Lighting.Colors.ColorConstants.LARSONGRAY);
}

public void setCHASERED() {
    lighting.set(Constants.Lighting.Colors.ColorConstants.CHASERED);
}

public void setCHASEBLUE() {
    lighting.set(Constants.Lighting.Colors.ColorConstants.CHASEBLUE);
}

public void setCHASEGRAY() {
    lighting.set(Constants.Lighting.Colors.ColorConstants.CHASEGRAY);
}

public void setHEARTBEATRED() {
    lighting.set(Constants.Lighting.Colors.ColorConstants.HEARTBEATRED);
}

public void setHEARTBEATBLUE() {
    lighting.set(Constants.Lighting.Colors.ColorConstants.HEARTBEATBLUE);
}

public void setHEARTBEATWHITE() {
    lighting.set(Constants.Lighting.Colors.ColorConstants.HEARTBEATWHITE);
}

public void setHEARTBEATGRAY() {
    lighting.set(Constants.Lighting.Colors.ColorConstants.HEARTBEATGRAY);
}

public void setBREATHRED() {
    lighting.set(Constants.Lighting.Colors.ColorConstants.BREATHRED);
}

public void setBREATHBLUE() {
    lighting.set(Constants.Lighting.Colors.ColorConstants.BREATHBLUE);
}

public void setBREATHGRAY() {
    lighting.set(Constants.Lighting.Colors.ColorConstants.BREATHGRAY);
}

public void setSTROBERED() {
    lighting.set(Constants.Lighting.Colors.ColorConstants.STROBERED);
}

public void setSTROBEBLUE() {
    lighting.set(Constants.Lighting.Colors.ColorConstants.STROBEBLUE);
}

public void setSTROBEGOLD() {
    lighting.set(Constants.Lighting.Colors.ColorConstants.STROBEGOLD);
}

public void setSTROBEWHITE() {
    lighting.set(Constants.Lighting.Colors.ColorConstants.STROBEWHITE);
}

public void setENDTOOFF() {
    lighting.set(Constants.Lighting.Colors.ColorConstants.ENDTOOFF);
}

public void setLARSONSCANNER() {
    lighting.set(Constants.Lighting.Colors.ColorConstants.LARSONSCANNER);
}

public void setLIGHTCHASE() {
    lighting.set(Constants.Lighting.Colors.ColorConstants.LIGHTCHASE);
}

public void setHEARTBEATSLOW() {
    lighting.set(Constants.Lighting.Colors.ColorConstants.HEARTBEATSLOW);
}

public void setHEARTBEATMEDIUM() {
    lighting.set(Constants.Lighting.Colors.ColorConstants.HEARTBEATMEDIUM);
}

public void setHEARTBEATFAST() {
    lighting.set(Constants.Lighting.Colors.ColorConstants.HEARTBEATFAST);
}

public void setBREATHSLOW() {
    lighting.set(Constants.Lighting.Colors.ColorConstants.BREATHSLOW);
}

public void setBREATHFAST() {
    lighting.set(Constants.Lighting.Colors.ColorConstants.BREATHFAST);
}

public void setSHOT() {
    lighting.set(Constants.Lighting.Colors.ColorConstants.SHOT);
}

public void setSTROBE() {
    lighting.set(Constants.Lighting.Colors.ColorConstants.STROBE);
}

public void setENDTOOFFTWO() {
    lighting.set(Constants.Lighting.Colors.ColorConstants.ENDTOOFFTWO);
}

public void setLARSONSCANNERTWO() {
    lighting.set(Constants.Lighting.Colors.ColorConstants.LARSONSCANNERTWO);
}

public void setLIGHTCHASETWO() {
    lighting.set(Constants.Lighting.Colors.ColorConstants.LIGHTCHASETWO);
}

public void setHEARTBEATSLOWTWO() {
    lighting.set(Constants.Lighting.Colors.ColorConstants.HEARTBEATSLOWTWO);
}

public void setHEARTBEATMEDIUMTWO() {
    lighting.set(Constants.Lighting.Colors.ColorConstants.HEARTBEATMEDIUMTWO);
}

public void setHEARTBEATFASTTWO() {
    lighting.set(Constants.Lighting.Colors.ColorConstants.HEARTBEATFASTTWO);
}

public void setBREATHSLOWYTWO() {
    lighting.set(Constants.Lighting.Colors.ColorConstants.BREATHSLOWYTWO);
}

public void setBREATHFASTTWO() {
    lighting.set(Constants.Lighting.Colors.ColorConstants.BREATHFASTTWO);
}

public void setSHOTTWO() {
    lighting.set(Constants.Lighting.Colors.ColorConstants.SHOTTWO);
}

public void setSTROBETWO() {
    lighting.set(Constants.Lighting.Colors.ColorConstants.STROBETWO);
}

public void setSPARKLEONEANDTWO() {
    lighting.set(Constants.Lighting.Colors.ColorConstants.SPARKLEONEANDTWO);
}

public void setSPARKLETWOANDONE() {
    lighting.set(Constants.Lighting.Colors.ColorConstants.SPARKLETWOANDONE);
}

public void setCOLORGRADIENTONEANDTWO() {
    lighting.set(Constants.Lighting.Colors.ColorConstants.COLORGRADIENTONEANDTWO);
}

public void setBEATSCOLORONEANDTWO() {
    lighting.set(Constants.Lighting.Colors.ColorConstants.BEATSCOLORONEANDTWO);
}

public void setENDTOENDOFF() {
    lighting.set(Constants.Lighting.Colors.ColorConstants.ENDTOENDOFF);
}

public void setENDTOEND() {
    lighting.set(Constants.Lighting.Colors.ColorConstants.ENDTOEND);
}

public void setCOLORONEANDCOLORTWO() {
    lighting.set(Constants.Lighting.Colors.ColorConstants.COLORONEANDCOLORTWO);
}

public void setTWINKLESCOLORS() {
    lighting.set(Constants.Lighting.Colors.ColorConstants.TWINKLESCOLORS);
}

public void setCOLORWAVESONEANDTWO() {
    lighting.set(Constants.Lighting.Colors.ColorConstants.COLORWAVESONEANDTWO);
}

public void setSINELONONEANDTWO() {
    lighting.set(Constants.Lighting.Colors.ColorConstants.SINELONONEANDTWO);
}

public void setHOTPINK() {
    lighting.set(Constants.Lighting.Colors.ColorConstants.HOTPINK);
}

public void setDARKRED() {
    lighting.set(Constants.Lighting.Colors.ColorConstants.DARKRED);
}

public void setRED() {
    lighting.set(Constants.Lighting.Colors.ColorConstants.RED);
}

public void setREDORANGE() {
    lighting.set(Constants.Lighting.Colors.ColorConstants.REDORANGE);
}

public void setORANGE() {
    lighting.set(Constants.Lighting.Colors.ColorConstants.ORANGE);
}

public void setGOLD() {
    lighting.set(Constants.Lighting.Colors.ColorConstants.GOLD);
}

public void setYELLOW() {
    lighting.set(Constants.Lighting.Colors.ColorConstants.YELLOW);
}

public void setLAWNGREEN() {
    lighting.set(Constants.Lighting.Colors.ColorConstants.LAWNGREEN);
}

public void setLIME() {
    lighting.set(Constants.Lighting.Colors.ColorConstants.LIME);
}

public void setDARKGREEN() {
    lighting.set(Constants.Lighting.Colors.ColorConstants.DARKGREEN);
}

public void setGREEN() {
    lighting.set(Constants.Lighting.Colors.ColorConstants.GREEN);
}

public void setBLUEGREEN() {
    lighting.set(Constants.Lighting.Colors.ColorConstants.BLUEGREEN);
}

public void setAQUA() {
    lighting.set(Constants.Lighting.Colors.ColorConstants.AQUA);
}

public void setSKYBLUE() {
    lighting.set(Constants.Lighting.Colors.ColorConstants.SKYBLUE);
}

public void setDARKBLUE() {
    lighting.set(Constants.Lighting.Colors.ColorConstants.DARKBLUE);
}

public void setBLUE() {
    lighting.set(Constants.Lighting.Colors.ColorConstants.BLUE);
}

public void setBLUEVIOLET() {
    lighting.set(Constants.Lighting.Colors.ColorConstants.BLUEVIOLET);
}

public void setVIOLET() {
    lighting.set(Constants.Lighting.Colors.ColorConstants.VIOLET);
}

public void setWHITE() {
    lighting.set(Constants.Lighting.Colors.ColorConstants.WHITE);
}

public void setGRAY() {
    lighting.set(Constants.Lighting.Colors.ColorConstants.GRAY);
}

public void setDARKGRAY() {
    lighting.set(Constants.Lighting.Colors.ColorConstants.DARKGRAY);
}

public void setOFF() {
    lighting.set(Constants.Lighting.Colors.ColorConstants.OFF);
}

  @Override
  public void periodic() {
    //This method will be called once per scheduler run
    // Colors selectedColor = lighting_chooser.getSelected();

    //lighting.set(selectedColor.getColorValue());
  }

  public static void set(double colorValue) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'set'");
  }
}
