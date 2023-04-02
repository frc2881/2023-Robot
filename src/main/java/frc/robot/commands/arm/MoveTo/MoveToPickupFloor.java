// Copyright (c) 2023 FRC Team 2881 - The Lady Cans
//
// Open Source Software; you can modify and/or share it under the terms of BSD
// license file in the root directory of this project.

package frc.robot.commands.arm.MoveTo;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.arm.ExtendArmToLength;
import frc.robot.commands.arm.TiltArmToHeight;
import frc.robot.commands.suction.EnableSuction;
import frc.robot.subsystems.ArmExtension;
import frc.robot.subsystems.ArmTilt;
import frc.robot.subsystems.Suction;

public class MoveToPickupFloor extends SequentialCommandGroup {

  public MoveToPickupFloor(ArmTilt armTilt, ArmExtension armExtension, Suction suction) {

    addCommands(
      new TiltArmToHeight(armTilt, 0.5, 3.14, false), // Bring arm out of robot
      new ExtendArmToLength(armExtension, 0.5, 16.0),
      new EnableSuction(suction) // Grab game piece

      
    );
  }
}
