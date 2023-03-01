// Copyright (c) 2022 FRC Team 2881 - The Lady Cans
//
// Open Source Software; you can modify and/or share it under the terms of BSD
// license file in the root directory of this project.

package frc.robot.commands.arm.MoveTo;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.commands.arm.TiltArmToHeight;
import frc.robot.subsystems.ArmExtension;
import frc.robot.subsystems.ArmTilt;

public class MoveToPickup extends SequentialCommandGroup {

  public MoveToPickup(
    ArmExtension armExtension, 
    ArmTilt armTilt, 
    Double speed
  ) {
    addCommands(
      new TiltArmToHeight(armTilt, speed, 12.6)
        .withTimeout(1.0)
    );
  }
  
}
