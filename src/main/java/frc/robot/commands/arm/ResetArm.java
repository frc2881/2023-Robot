// Copyright (c) 2023 FRC Team 2881 - The Lady Cans
//
// Open Source Software; you can modify and/or share it under the terms of BSD
// license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.commands.controllers.RumbleController;
import frc.robot.commands.controllers.RumbleController.RumblePattern;
import frc.robot.subsystems.ArmExtension;
import frc.robot.subsystems.ArmTilt;

public class ResetArm extends SequentialCommandGroup {
  
  public ResetArm(ArmExtension armExtension, ArmTilt armTilt, Double speed) {
    addCommands(
      new ParallelCommandGroup(
        new ExtendArmToLength(armExtension, speed, Constants.Arm.kExtendReverseLimit).withTimeout(Constants.Arm.kExtensionTimeOut),
        new SequentialCommandGroup(
          new WaitUntilCommand(() -> armExtension.getEncoderPosition() <= 15),
          new TiltArmToHeight(armTilt, speed, Constants.Arm.kTiltReverseLimit, false)
            .withTimeout(Constants.Arm.kTiltTimeOut))
        )
    );
  }
}
