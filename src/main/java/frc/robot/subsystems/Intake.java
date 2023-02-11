// Copyright (c) 2023 FRC Team 2881 - The Lady Cans
//
// Open Source Software; you can modify and/or share it under the terms of BSD
// license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  public boolean isOut = true;
  /** Creates a new Intake. */
  public Intake() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public boolean isOut(){
    return isOut;
  }
}
