// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

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

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreHighCone extends SequentialCommandGroup {
  /** Creates a new ScoreHighCone. */
  public ScoreHighCone(
    ArmExtension armExtension, 
    ArmTilt armTilt, 
    double speed
  ) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ConditionalCommand(
        new ExtendArmToLength(armExtension, speed, Constants.Arm.kExtendReverseLimit)
          .withTimeout(Constants.Arm.kExtensionTimeOut),
        new WaitCommand(0.001), 
        () -> (armExtension.getEncoderPosition() > Constants.Arm.kExtendReverseLimit)),

      new ParallelRaceGroup(
        new TiltArmToHeight(armTilt, speed, 16.0, true),
        new SequentialCommandGroup(
          new WaitUntilCommand(() -> armTilt.getEncoderPosition() >= 10.0),
          new ExtendArmToLength(armExtension, speed, 28.0)
            .withTimeout(Constants.Arm.kExtensionTimeOut))
        ),
      new TiltArmToHeight(armTilt, speed * 0.5, 14.5, false).withTimeout(Constants.Arm.kTiltTimeOut)
    );
  }
}
