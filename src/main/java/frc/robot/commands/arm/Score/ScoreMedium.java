// Copyright (c) 2022 FRC Team 2881 - The Lady Cans
//
// Open Source Software; you can modify and/or share it under the terms of BSD
// license file in the root directory of this project.

package frc.robot.commands.arm.Score;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.lib.Node.NodeType;
import frc.robot.subsystems.ArmExtension;
import frc.robot.subsystems.ArmTilt;
import frc.robot.subsystems.Drive;

public class ScoreMedium extends SequentialCommandGroup {
  
  public ScoreMedium(
    ArmExtension armExtension, 
    ArmTilt armTilt, 
    Drive drive,
    Double speed
  ) {
    addCommands(
      new ConditionalCommand(
        new ScoreMediumCone(armExtension, armTilt, speed), 
        new ScoreMediumCube(armExtension, armTilt, speed), 
        () -> drive.getNearesNodeType() == NodeType.CONE)
      
    );
  }
  
}
