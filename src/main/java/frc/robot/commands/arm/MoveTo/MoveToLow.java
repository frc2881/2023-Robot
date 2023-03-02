// Copyright (c) 2022 FRC Team 2881 - The Lady Cans
//
// Open Source Software; you can modify and/or share it under the terms of BSD
// license file in the root directory of this project.

package frc.robot.commands.arm.MoveTo;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.commands.arm.ExtendArmToLength;
import frc.robot.commands.arm.TiltArmToHeight;
import frc.robot.subsystems.ArmExtension;
import frc.robot.subsystems.ArmTilt;

public class MoveToLow extends SequentialCommandGroup {

  public MoveToLow(
    ArmExtension armExtension, 
    ArmTilt armTilt, 
    Double speed
  ) {
    addCommands(
      new ExtendArmToLength(armExtension, speed, 0.0)
        .withTimeout(2.5), 
      new TiltArmToHeight(armTilt, speed, 6.0)
        .withTimeout(2.0),
      new ExtendArmToLength(armExtension, speed, 10.5)
        .withTimeout(2.5)
    );
  }
  
}
