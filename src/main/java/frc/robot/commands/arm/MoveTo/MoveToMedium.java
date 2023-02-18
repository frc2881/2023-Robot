// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm.MoveTo;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.arm.ExtendArmToLength;
import frc.robot.commands.arm.TiltArmToHeight;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class MoveToMedium extends SequentialCommandGroup {
  /** Creates a new MoveToMedium. */
  public MoveToMedium(Arm arm, Intake intake, Double speed) {
    // Based on Cone nodes
    addCommands(new TiltArmToHeight(arm, intake, speed, 15.0),
    new ExtendArmToLength(arm, speed, 13.0));

  }
}
