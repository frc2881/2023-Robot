// Copyright (c) 2022 FRC Team 2881 - The Lady Cans
//
// Open Source Software; you can modify and/or share it under the terms of BSD
// license file in the root directory of this project.

package frc.robot.commands.arm.MoveTo;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.arm.TiltArmToHeight;
import frc.robot.commands.suction.EnableSuction;
import frc.robot.subsystems.ArmExtension;
import frc.robot.subsystems.ArmTilt;
import frc.robot.subsystems.Suction;

public class MoveToPickupSubstation extends ParallelCommandGroup {

  public MoveToPickupSubstation(
    ArmExtension armExtension, 
    ArmTilt armTilt, 
    Double speed,
    Suction suction
  ) {
    addCommands(
      new EnableSuction(suction),
      new SequentialCommandGroup(
        new ParallelRaceGroup(
          new TiltArmToHeight(armTilt, speed, 12.6, true),
          new WaitUntilCommand(() -> suction.isVacuumEnabled())),
        new TiltArmToHeight(armTilt, speed, 13.5, true)
      ).withTimeout(10.0)
    );
  }
}