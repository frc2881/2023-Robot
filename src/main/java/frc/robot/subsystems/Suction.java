// Copyright (c) 2023 FRC Team 2881 - The Lady Cans
//
// Open Source Software; you can modify and/or share it under the terms of BSD
// license file in the root directory of this project.

package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Suction extends SubsystemBase {
  private final CANSparkMax m_motor;
  private final Solenoid m_solenoid;

  public Suction() {
    m_motor = new CANSparkMax(Constants.Suction.kMotorId, MotorType.kBrushless);
    m_motor.restoreFactoryDefaults();
    m_motor.setInverted(false);
    m_motor.setIdleMode(IdleMode.kBrake);
    m_motor.setSmartCurrentLimit(Constants.Suction.kCurrentLimit);

    m_solenoid = new Solenoid(PneumaticsModuleType.REVPH, Constants.Suction.kSolenoidId);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void run(double speed) {
    m_solenoid.set(false);
    m_motor.set(speed > Constants.Suction.kMaxSpeed ? Constants.Suction.kMaxSpeed : speed);
  }

  public void release() {
    m_motor.set(0);
    m_solenoid.set(true);
  }

}
