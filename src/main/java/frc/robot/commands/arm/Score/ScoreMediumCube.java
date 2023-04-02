// Copyright (c) 2023 FRC Team 2881 - The Lady Cans
//
// Open Source Software; you can modify and/or share it under the terms of BSD
// license file in the root directory of this project.

package frc.robot.commands.arm.Score;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.commands.arm.ExtendArmToLength;
import frc.robot.commands.arm.TiltArmToHeight;
import frc.robot.subsystems.ArmExtension;
import frc.robot.subsystems.ArmTilt;

public class ScoreMediumCube extends SequentialCommandGroup {

  public ScoreMediumCube(
    ArmExtension armExtension, 
    ArmTilt armTilt, 
    Double speed
  ) {
    addCommands(
      new ConditionalCommand(
        new ExtendArmToLength(armExtension, speed, Constants.Arm.kExtendReverseLimit)
          .withTimeout(Constants.Arm.kExtensionTimeOut),
          new WaitCommand(0.001), 
        () -> (armExtension.getEncoderPosition() > Constants.Arm.kExtendReverseLimit)),
    
      new ParallelRaceGroup(
        new TiltArmToHeight(armTilt, speed, 10.5, true),
        new SequentialCommandGroup(
          new WaitUntilCommand(() -> armTilt.getEncoderPosition() >= 9.0),
          new ExtendArmToLength(armExtension, speed, 3.0)
            .withTimeout(Constants.Arm.kExtensionTimeOut)))
    );
  }
  
}
