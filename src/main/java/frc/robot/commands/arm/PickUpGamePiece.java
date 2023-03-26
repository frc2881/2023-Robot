// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.suction.EnableSuction;
import frc.robot.subsystems.ArmExtension;
import frc.robot.subsystems.ArmTilt;
import frc.robot.subsystems.Suction;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PickUpGamePiece extends SequentialCommandGroup {
  /** Creates a new PickUpGamePiece. */
  public PickUpGamePiece(ArmTilt armTilt, ArmExtension armExtension, Suction suction) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new TiltArmToHeight(armTilt, 0.5, null, false), // Bring arm out of robot
      new ExtendArmToLength(armExtension, 0.5, null),
      new EnableSuction(suction), // Grab game piece
      new WaitUntilCommand(suction::isVacuumEnabled).withTimeout(2.0),
      new TiltArmToHeight(armTilt, 0.5, null, false), // Bring gamepiece up and into the robot
      new ExtendArmToLength(armExtension, 0.5, 0.0)
    );
  }
}
