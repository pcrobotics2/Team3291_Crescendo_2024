// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;
import frc.robot.commands.Auto.AutoDirectives.BL.BL_BackOut;
import frc.robot.commands.Auto.AutoDirectives.BL.BL_ToSpeaker;
import frc.robot.commands.LaunchNoteCMD;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.LauncherSub;
import frc.robot.subsystems.IntakeMotorSubsystem;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BL_MildAuto extends SequentialCommandGroup {
  /** Creates a new BL_MildAuto. */
  public BL_MildAuto(SwerveSubsystem swerveSubsystem, LauncherSub launcherSub, IntakeMotorSubsystem intakeMotorSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new BL_ToSpeaker(swerveSubsystem),
      new LaunchNoteCMD(intakeMotorSubsystem, launcherSub).withTimeout(5),
      new BL_BackOut(swerveSubsystem)
    );
  }
}