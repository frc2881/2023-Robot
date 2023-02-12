// Copyright (c) 2023 FRC Team 2881 - The Lady Cans
//
// Open Source Software; you can modify and/or share it under the terms of BSD
// license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.RobotController;

import frc.robot.Constants;
import frc.robot.lib.Utils;

public class Suction extends SubsystemBase {
  private final AnalogInput m_AnalogInputPressureBottom;
  private final AnalogInput m_AnalogInputPressureTop;
  private final CANSparkMax m_motorBottom;
  private final CANSparkMax m_motorTop;
  private final Solenoid m_solenoidBottom;
  private final Solenoid m_solenoidTop;
  private boolean m_isEnabled = false;
  private boolean m_isTargetPressureBottomReached = false;
  private boolean m_isTargetPressureTopReached = false;
  

  public Suction() {
    m_AnalogInputPressureBottom = new AnalogInput(Constants.Suction.kPressureSensorBottomId);
    m_AnalogInputPressureTop = new AnalogInput(Constants.Suction.kPressureSensorTopId);

    m_motorBottom = new CANSparkMax(Constants.Suction.kMotorBottomId, MotorType.kBrushless);
    m_motorBottom.restoreFactoryDefaults();
    m_motorBottom.setInverted(false);
    m_motorBottom.setIdleMode(IdleMode.kBrake);
    m_motorBottom.setSmartCurrentLimit(Constants.Suction.kCurrentLimit);

    m_motorTop = new CANSparkMax(Constants.Suction.kMotorTopId, MotorType.kBrushless);
    m_motorTop.restoreFactoryDefaults();
    m_motorTop.setInverted(false);
    m_motorTop.setIdleMode(IdleMode.kBrake);
    m_motorTop.setSmartCurrentLimit(Constants.Suction.kCurrentLimit);

    m_solenoidBottom = new Solenoid(PneumaticsModuleType.REVPH, Constants.Suction.kSolenoidBottomId);
    m_solenoidTop = new Solenoid(PneumaticsModuleType.REVPH, Constants.Suction.kSolenoidTopId);

  }

  @Override
  public void periodic() {
    double supplyVoltage = RobotController.getVoltage5V();
    double pressureBottom = Utils.voltsToPsi(m_AnalogInputPressureBottom.getAverageVoltage(), supplyVoltage);
    double pressureTop = Utils.voltsToPsi(m_AnalogInputPressureTop.getAverageVoltage(), supplyVoltage);

    //Automated reenabling of the suction system
    if (m_isEnabled) {
      m_solenoidBottom.set(false);
      m_solenoidTop.set(false);
      
      //We seperated the sensor statements so the motors only run when necessary and not when the individual one has enough pressure
      if (!m_isTargetPressureBottomReached) {
        if (pressureBottom > Constants.Suction.kTargetPressureBottom) {
          m_motorBottom.set(Constants.Suction.kMaxSpeed);
        } else {
          m_motorBottom.set(0);
          m_isTargetPressureBottomReached = true;
        }
      } else {
        if (pressureBottom > Constants.Suction.kMinimumPressureBottom) {
          m_isTargetPressureBottomReached = false;
        }
      } 

      if (!m_isTargetPressureTopReached) {
        if (pressureTop > Constants.Suction.kTargetPressureTop) {
          m_motorTop.set(Constants.Suction.kMaxSpeed);
        } else {
          m_motorTop.set(0);
          m_isTargetPressureTopReached = true;
        }
      } else {
        if (pressureTop > Constants.Suction.kMinimumPressureTop) {
          m_isTargetPressureTopReached = false;
        }
      } 
      
    } else {
      m_motorBottom.set(0);
      m_motorTop.set(0);
      m_solenoidBottom.set(true);
      m_solenoidTop.set(true);
      m_isTargetPressureBottomReached = false;
      m_isTargetPressureTopReached = false;
    }

    SmartDashboard.putNumber("Suction/PressureBottom", pressureBottom);
    SmartDashboard.putNumber("Suction/PressureTop", pressureTop);
  }

  public void enable() {  
    m_isEnabled = true;
  }

  public void disable() {
    m_isEnabled = false;
  }
}
