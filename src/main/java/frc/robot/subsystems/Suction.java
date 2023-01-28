// Copyright (c) 2023 FRC Team 2881 - The Lady Cans
//
// Open Source Software; you can modify and/or share it under the terms of BSD
// license file in the root directory of this project.

package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Suction extends SubsystemBase {
  
  private final PneumaticHub m_pneumaticHub = new PneumaticHub();
  private final CANSparkMax m_motor1;
  //private final CANSparkMax m_motor2;
  private final Solenoid m_solenoid1;
  //private final Solenoid m_solenoid2;

  public Suction() {
    m_motor1 = new CANSparkMax(Constants.Suction.kMotor1Id, MotorType.kBrushless);
    m_motor1.restoreFactoryDefaults();
    m_motor1.setInverted(false);
    m_motor1.setIdleMode(IdleMode.kBrake);
    m_motor1.setSmartCurrentLimit(Constants.Suction.kCurrentLimit);

    //TODO We eventually will have two solenoids and two motors for the mechanism 
    //m_motor2 = new CANSparkMax(Constants.Suction.kMotorId, MotorType.kBrushless);
    //m_motor2.restoreFactoryDefaults();
    //m_motor2.setInverted(false);
    //m_motor2.setIdleMode(IdleMode.kBrake);
    //m_motor2.setSmartCurrentLimit(Constants.Suction.kCurrentLimit);

    m_solenoid1 = new Solenoid(PneumaticsModuleType.REVPH, Constants.Suction.kSolenoid1Id);
    //m_solenoid2 = new Solenoid(PneumaticsModuleType.REVPH, Constants.Suction.kSolenoidId2);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void run(double speed) {
    speed = speed > Constants.Suction.kMaxSpeed ? Constants.Suction.kMaxSpeed : speed;
    m_solenoid1.set(false);
    //m_solenoid2.set(false);
    m_motor1.set(speed);
    //m_motor2.set(speed);
  }

  public void release() {
    m_motor1.set(0);
    //m_motor2.set(0);
    m_solenoid1.set(true);
    //m_solenoid2.set(true);
  }

  public void reset() {
    m_motor1.set(0);
    //m_motor2.set(0);
    m_solenoid1.set(false);
    //m_solenoid2.set(false);
  }

}
