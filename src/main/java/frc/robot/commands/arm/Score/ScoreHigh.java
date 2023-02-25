// Copyright (c) 2022 FRC Team 2881 - The Lady Cans
//
// Open Source Software; you can modify and/or share it under the terms of BSD
// license file in the root directory of this project.

package frc.robot.commands.arm.Score;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.commands.arm.TiltArmToHeight;
import frc.robot.commands.arm.MoveTo.MoveToHigh;
import frc.robot.subsystems.ArmExtension;
import frc.robot.subsystems.ArmTilt;
import frc.robot.subsystems.Suction;

public class ScoreHigh extends SequentialCommandGroup {

  public ScoreHigh(ArmExtension armExtension, ArmTilt armTilt, Double speed, Suction suction) {
    addCommands(new MoveToHigh(armExtension, armTilt,  speed),
    new TiltArmToHeight(armTilt, speed, 14.5));
  }
  
}
