// Copyright (c) 2023 FRC Team 2881 - The Lady Cans
//
// Open Source Software; you can modify and/or share it under the terms of BSD
// license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
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
  private boolean m_isDisabling = false;
  private boolean m_isTargetPressureBottomReached = false;
  private boolean m_isTargetPressureTopReached = false;
  private double m_currentPressureBottom = 0;
  private double m_currentPressureTop = 0;

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

    SmartDashboard.putNumber("Suction/Bottom/Pressure/Minimum", Constants.Suction.kMinimumPressureBottom);
    SmartDashboard.putNumber("Suction/Bottom/Pressure/Target", Constants.Suction.kTargetPressureBottom);
    SmartDashboard.putNumber("Suction/Top/Pressure/Minimum", Constants.Suction.kMinimumPressureTop);
    SmartDashboard.putNumber("Suction/Top/Pressure/Target", Constants.Suction.kTargetPressureTop);
  }

  @Override
  public void periodic() {
    double supplyVoltage = RobotController.getVoltage5V();
    m_currentPressureBottom = Utils.voltsToPsi(m_AnalogInputPressureBottom.getAverageVoltage(), supplyVoltage);
    m_currentPressureTop = Utils.voltsToPsi(m_AnalogInputPressureTop.getAverageVoltage(), supplyVoltage);

    //Automated reenabling of the suction system
    if (m_isEnabled) {
      m_solenoidBottom.set(false);
      m_solenoidTop.set(false);
      
      //We seperated the sensor statements so the motors only run when necessary and not when the individual one has enough pressure
      if (!m_isTargetPressureBottomReached) {
        if (m_currentPressureBottom > Constants.Suction.kTargetPressureBottom) {
          m_motorBottom.set(Constants.Suction.kMaxSpeed);
        } else {
          m_motorBottom.set(0);
          m_isTargetPressureBottomReached = true;
        }
      } else {
        if (m_currentPressureBottom > Constants.Suction.kMinimumPressureBottom) {
          m_isTargetPressureBottomReached = false;
        }
      } 

      if (!m_isTargetPressureTopReached) {
        if (m_currentPressureTop > Constants.Suction.kTargetPressureTop) {
          m_motorTop.set(Constants.Suction.kMaxSpeed);
        } else {
          m_motorTop.set(0);
          m_isTargetPressureTopReached = true;
        }
      } else {
        if (m_currentPressureTop > Constants.Suction.kMinimumPressureTop) {
          m_isTargetPressureTopReached = false;
        }
      } 
      
    } else {
      if (m_isDisabling){
        m_motorTop.set(Constants.Suction.kMaxSpeed);
        m_motorBottom.set(0);
        m_solenoidBottom.set(true);
        Timer.delay(Constants.Suction.kDelay);
        m_motorTop.set(0);
        m_solenoidTop.set(true);
        m_isTargetPressureBottomReached = false;
        m_isTargetPressureTopReached = false;
        m_isDisabling = false;
      } else {
        reset();
      }
    } 

    updateTelemetry();
  }
  
  public void enable() {  
    m_isEnabled = true;
    SmartDashboard.putBoolean("Suction/IsEnabled", m_isEnabled);
  }

  public void disable() {
    m_isEnabled = false;
    m_isDisabling = true;
    SmartDashboard.putBoolean("Suction/IsEnabled", m_isEnabled);
  }

  public void toggle() {
    if (m_isEnabled) {
      disable();
    } else {
      enable();
    }
  }

  public void reset() {
    m_isEnabled = false;
    m_isDisabling = false;
    m_motorBottom.set(0);
    m_motorTop.set(0);
    SmartDashboard.putBoolean("Suction/IsEnabled", m_isEnabled);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);

    builder.addDoubleProperty("Bottom/Motor/Speed", m_motorBottom::get, null);
    builder.addDoubleProperty("Top/Motor/Speed", m_motorTop::get, null);
  }

  private void updateTelemetry() {
    SmartDashboard.putNumber("Suction/Bottom/Pressure/Current", m_currentPressureBottom);
    SmartDashboard.putNumber("Suction/Top/Pressure/Current", m_currentPressureTop);
  }
}
