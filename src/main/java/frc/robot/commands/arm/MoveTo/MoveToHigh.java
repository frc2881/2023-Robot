// Copyright (c) 2022 FRC Team 2881 - The Lady Cans
//
// Open Source Software; you can modify and/or share it under the terms of BSD
// license file in the root directory of this project.

package frc.robot.commands.arm.MoveTo;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.arm.ExtendArmToLength;
import frc.robot.commands.arm.TiltArmToHeight;
import frc.robot.subsystems.ArmExtension;
import frc.robot.subsystems.ArmTilt;

public class MoveToHigh extends SequentialCommandGroup {

  public MoveToHigh(
    ArmExtension armExtension, 
    ArmTilt armTilt, 
    Double speed
  ) {

    addCommands(
      new ConditionalCommand(
        new ExtendArmToLength(armExtension, speed, Constants.Arm.kExtensionResetPosition)
          .withTimeout(Constants.Arm.kExtensionTimeOut),
        new WaitCommand(0.001), 
        () -> (armExtension.getEncoderPosition() > Constants.Arm.kExtensionResetPosition)),

      new TiltArmToHeight(armTilt, speed, 10.0)
        .withTimeout(Constants.Arm.kTiltTimeOut),
      new ParallelCommandGroup(
        new TiltArmToHeight(armTilt, speed, 15.5)
          .withTimeout(Constants.Arm.kTiltTimeOut),
        new ExtendArmToLength(armExtension, speed, 27.5)
          .withTimeout(Constants.Arm.kExtensionTimeOut))
    );
  }
  
}
