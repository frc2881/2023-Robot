// Copyright (c) 2023 FRC Team 2881 - The Lady Cans
//
// Open Source Software; you can modify and/or share it under the terms of BSD
// license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  private final CANSparkMax m_rollers;
  private final CANSparkMax m_arm;
  /** Creates a new Intake. */
  public Intake() {
    m_rollers = new CANSparkMax(Constants.Intake.kIntakeRollersCANId, MotorType.kBrushless);
    m_arm = new CANSparkMax(Constants.Intake.kIntakeArmCANId, MotorType.kBrushless);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void runRollers(double speed){
    m_rollers.set(speed);
  }

  public void moveArm(double speed){
    m_arm.set(speed); // Presets: All the way out, all the way in and somewhere in the middle to let the arm out
  }
}
