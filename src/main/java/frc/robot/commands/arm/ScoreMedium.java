// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.intake.ExtendIntakeArm;
import frc.robot.commands.suction.DisableSuction;
import frc.robot.commands.suction.EnableSuction;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Suction;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreMedium extends SequentialCommandGroup {
  
  public ScoreMedium(Arm arm, Intake intake, Double speed, Suction suction) {
    /* 
    if(intake.isOut = false) {
      addCommands(new ExtendIntakeArm(intake));
    } */
    //13.5, 13, 11.8
    addCommands(
      new TiltArmToHeight(arm, intake, speed, 1.0),
      new ExtendArmToLength(arm, speed, 4.5),
      new EnableSuction(suction));
      /*
      new ExtendArmToLength(arm, speed, 0.1),
      new TiltArmToHeight(arm, intake, speed, 13.5),
      new ExtendArmToLength(arm, speed, 13.0),
      new TiltArmToHeight(arm, intake, speed, 11.8),
      new DisableSuction(suction)); */ // Don't release at end
  }
}
