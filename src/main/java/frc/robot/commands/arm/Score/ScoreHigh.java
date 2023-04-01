// Copyright (c) 2022 FRC Team 2881 - The Lady Cans
//
// Open Source Software; you can modify and/or share it under the terms of BSD
// license file in the root directory of this project.

package frc.robot.commands.arm.Score;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.arm.TiltArmToHeight;
import frc.robot.commands.arm.MoveTo.MoveToHigh;
import frc.robot.subsystems.ArmExtension;
import frc.robot.subsystems.ArmTilt;
import frc.robot.subsystems.Suction;

public class ScoreHigh extends SequentialCommandGroup {

  public ScoreHigh(
    ArmExtension armExtension, 
    ArmTilt armTilt, 
    double speed, 
    Suction suction,
    boolean isCube
  ) {
    addCommands(
      new MoveToHigh(armExtension, armTilt, speed),
      new TiltArmToHeight(armTilt, speed * 0.5, 14.5, false)
      .withTimeout(Constants.Arm.kTiltTimeOut)

      
    );
  }
  
}
