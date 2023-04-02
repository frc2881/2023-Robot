// Copyright (c) 2023 FRC Team 2881 - The Lady Cans
//
// Open Source Software; you can modify and/or share it under the terms of BSD
// license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.suction.EnableSuction;
import frc.robot.subsystems.ArmExtension;
import frc.robot.subsystems.ArmTilt;
import frc.robot.subsystems.Suction;

public class PickUpCone extends SequentialCommandGroup {

  public PickUpCone(ArmTilt armTilt, ArmExtension armExtension, Suction suction) {

    addCommands(
      new TiltArmToHeight(armTilt, 0.5, 3.14, false), // Bring arm out of robot
      new ExtendArmToLength(armExtension, 0.5, 16.0),
      new EnableSuction(suction) // Grab game piece
      /*new WaitUntilCommand(suction::isVacuumEnabledForCone).withTimeout(2.0),
      new TiltArmToHeight(armTilt, 0.5, 6.0, false), // Bring gamepiece up and into the robot
      new ExtendArmToLength(armExtension, 0.5, 0.0)*/
    );
  }
  
}
