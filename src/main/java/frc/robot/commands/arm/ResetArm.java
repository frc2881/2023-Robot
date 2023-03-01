// Copyright (c) 2023 FRC Team 2881 - The Lady Cans
//
// Open Source Software; you can modify and/or share it under the terms of BSD
// license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmExtension;
import frc.robot.subsystems.ArmTilt;

public class ResetArm extends SequentialCommandGroup {
  
  public ResetArm(ArmExtension armExtension, ArmTilt armTilt, Double speed) {
    addCommands(
      new TiltArmToHeight(armTilt, speed, 14.0)
        .withTimeout(1.0),
      new ExtendArmToLength(armExtension, speed, 0.0)
        .withTimeout(2.5), 
      new TiltArmToHeight(armTilt, speed, 0.0)
        .withTimeout(2.5)
    );
  }
}
