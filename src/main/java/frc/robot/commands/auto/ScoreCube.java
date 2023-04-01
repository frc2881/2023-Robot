// Copyright (c) 2023 FRC Team 2881 - The Lady Cans
//
// Open Source Software; you can modify and/or share it under the terms of BSD
// license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.arm.Score.ScoreHighCube;
import frc.robot.commands.suction.DisableSuction;
import frc.robot.commands.suction.EnableSuction;
import frc.robot.subsystems.ArmExtension;
import frc.robot.subsystems.ArmTilt;
import frc.robot.subsystems.Suction;

public class ScoreCube extends SequentialCommandGroup {

  public ScoreCube(
    Suction suction, 
    ArmExtension armExtension, 
    ArmTilt armTilt
  ) {
    addCommands(
      new WaitCommand(0.02),
      new EnableSuction(suction),
      new WaitUntilCommand(suction::isVacuumEnabledForCube).withTimeout(4.0),
      new ScoreHighCube(armExtension, armTilt, 1.0),
      new DisableSuction(suction)
    );
  }

}
