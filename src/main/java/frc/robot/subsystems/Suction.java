// Copyright (c) 2023 FRC Team 2881 - The Lady Cans
//
// Open Source Software; you can modify and/or share it under the terms of BSD
// license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.lib.Utils;

public class Suction extends SubsystemBase {
  private final AnalogInput m_AnalogInputPressureBottom;
  private final AnalogInput m_AnalogInputPressureTop;
  private final AnalogInput m_AnalogInputPressureLeft;
  private final AnalogInput m_AnalogInputPressureRight;
  private final CANSparkMax m_motorBottom;
  private final CANSparkMax m_motorTop;
  private final CANSparkMax m_motorLeft;
  private final CANSparkMax m_motorRight;
  private final Solenoid m_solenoidBottom;
  private final Solenoid m_solenoidTop;
  private final Solenoid m_solenoidLeft;
  private final Solenoid m_solenoidRight;
  private boolean m_isEnabled = false;
  private boolean m_isDisabling = false;
  private boolean m_isMaximumPressureBottomReached = false;
  private boolean m_isMaximumPressureTopReached = false;
  private boolean m_isMaximumPressureLeftReached = false;
  private boolean m_isMaximumPressureRightReached = false;
  private double m_currentPressureBottom = 0;
  private double m_currentPressureTop = 0;
  private double m_currentPressureLeft = 0;
  private double m_currentPressureRight = 0;

  private DoubleLogEntry m_logSuctionBottomPressure;
  private DoubleLogEntry m_logSuctionBottomOutput;
  private DoubleLogEntry m_logSuctionBottomBusVoltage;
  private DoubleLogEntry m_logSuctionBottomCurrent;

  private DoubleLogEntry m_logSuctionTopPressure; 
  private DoubleLogEntry m_logSuctionTopOutput;
  private DoubleLogEntry m_logSuctionTopBusVoltage;
  private DoubleLogEntry m_logSuctionTopCurrent;
  
  private DoubleLogEntry m_logSuctionLeftPressure; 
  private DoubleLogEntry m_logSuctionLeftOutput;
  private DoubleLogEntry m_logSuctionLeftBusVoltage;
  private DoubleLogEntry m_logSuctionLeftCurrent;

  private DoubleLogEntry m_logSuctionRightPressure; 
  private DoubleLogEntry m_logSuctionRightOutput;
  private DoubleLogEntry m_logSuctionRightBusVoltage;
  private DoubleLogEntry m_logSuctionRightCurrent;

  public Suction() {
    m_AnalogInputPressureBottom = new AnalogInput(Constants.Suction.kPressureSensorBottomId);
    m_AnalogInputPressureTop = new AnalogInput(Constants.Suction.kPressureSensorTopId);
    m_AnalogInputPressureLeft = new AnalogInput(Constants.Suction.kPressureSensorLeftId);
    m_AnalogInputPressureRight = new AnalogInput(Constants.Suction.kPressureSensorRightId);

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
    
    m_motorLeft = new CANSparkMax(Constants.Suction.kMotorLeftId, MotorType.kBrushless);
    m_motorLeft.restoreFactoryDefaults();
    m_motorLeft.setInverted(false);
    m_motorLeft.setIdleMode(IdleMode.kBrake);
    m_motorLeft.setSmartCurrentLimit(Constants.Suction.kCurrentLimit);

    m_motorRight = new CANSparkMax(Constants.Suction.kMotorRightId, MotorType.kBrushless);
    m_motorRight.restoreFactoryDefaults();
    m_motorRight.setInverted(false);
    m_motorRight.setIdleMode(IdleMode.kBrake);
    m_motorRight.setSmartCurrentLimit(Constants.Suction.kCurrentLimit);

    m_solenoidBottom = new Solenoid(PneumaticsModuleType.REVPH, Constants.Suction.kSolenoidBottomId);
    m_solenoidTop = new Solenoid(PneumaticsModuleType.REVPH, Constants.Suction.kSolenoidTopId);
    m_solenoidLeft = new Solenoid(PneumaticsModuleType.REVPH, Constants.Suction.kSolenoidLeftId);
    m_solenoidRight = new Solenoid(PneumaticsModuleType.REVPH, Constants.Suction.kSolenoidRightId);

    DataLog log = DataLogManager.getLog();
    m_logSuctionBottomPressure = new DoubleLogEntry(log, "/suction/bottom/pressure");
    m_logSuctionBottomOutput = new DoubleLogEntry(log, "/suction/bottom/output");
    m_logSuctionBottomBusVoltage = new DoubleLogEntry(log, "/suction/bottom/busVoltage");
    m_logSuctionBottomCurrent = new DoubleLogEntry(log, "/suction/bottom/current");

    m_logSuctionTopPressure = new DoubleLogEntry(log, "/suction/top/pressure");
    m_logSuctionTopOutput = new DoubleLogEntry(log, "/suction/top/output");
    m_logSuctionTopBusVoltage = new DoubleLogEntry(log, "/suction/top/busVoltage");
    m_logSuctionTopCurrent = new DoubleLogEntry(log, "/suction/top/current");

    m_logSuctionLeftPressure = new DoubleLogEntry(log, "/suction/left/pressure");
    m_logSuctionLeftOutput = new DoubleLogEntry(log, "/suction/left/output");
    m_logSuctionLeftBusVoltage = new DoubleLogEntry(log, "/suction/left/busVoltage");
    m_logSuctionLeftCurrent = new DoubleLogEntry(log, "/suction/left/current");

    m_logSuctionRightPressure = new DoubleLogEntry(log, "/suction/right/pressure");
    m_logSuctionRightOutput = new DoubleLogEntry(log, "/suction/right/output");
    m_logSuctionRightBusVoltage = new DoubleLogEntry(log, "/suction/right/busVoltage");
    m_logSuctionRightCurrent = new DoubleLogEntry(log, "/suction/right/current");
    
    SmartDashboard.putNumber("Suction/Pressure/Minimum", Constants.Suction.kMinimumPressure);
    SmartDashboard.putNumber("Suction/Pressure/Target", Constants.Suction.kTargetPressure);
    SmartDashboard.putNumber("Suction/Pressure/Maximum", Constants.Suction.kMaximumPressure);
  }

  @Override
  public void periodic() {
    double supplyVoltage = RobotController.getVoltage5V();
    m_currentPressureBottom = Utils.voltsToPsi(m_AnalogInputPressureBottom.getAverageVoltage(), supplyVoltage);
    m_currentPressureTop = Utils.voltsToPsi(m_AnalogInputPressureTop.getAverageVoltage(), supplyVoltage);
    m_currentPressureLeft = Utils.voltsToPsi(m_AnalogInputPressureLeft.getAverageVoltage(), supplyVoltage);
    m_currentPressureRight = Utils.voltsToPsi(m_AnalogInputPressureRight.getAverageVoltage(), supplyVoltage);

    //Automated reenabling of the suction system
    if (m_isEnabled) {
      m_solenoidBottom.set(false);
      m_solenoidTop.set(false);
      m_solenoidLeft.set(false);
      m_solenoidRight.set(false);
      
      //We seperated the sensor statements so the motors only run when necessary and not when the individual one has enough pressure
      if (!m_isMaximumPressureBottomReached) {
        if (m_currentPressureBottom > Constants.Suction.kMaximumPressure) {
          m_motorBottom.set(Constants.Suction.kMaxSpeed);
        } else {
          m_motorBottom.set(0);
          m_isMaximumPressureBottomReached = true;
        }
      } else {
        if (m_currentPressureBottom > Constants.Suction.kTargetPressure) {
          m_isMaximumPressureBottomReached = false;
        }
      } 

      if (!m_isMaximumPressureTopReached) {
        if (m_currentPressureTop > Constants.Suction.kMaximumPressure) {
          m_motorTop.set(Constants.Suction.kMaxSpeed);
        } else {
          m_motorTop.set(0);
          m_isMaximumPressureTopReached = true;
        }
      } else {
        if (m_currentPressureTop > Constants.Suction.kTargetPressure) {
          m_isMaximumPressureTopReached = false;
        }
      } 

      if (!m_isMaximumPressureLeftReached) {
        if (m_currentPressureLeft > Constants.Suction.kMaximumPressure) {
          m_motorLeft.set(Constants.Suction.kMaxSpeed);
        } else {
          m_motorLeft.set(0);
          m_isMaximumPressureLeftReached = true;
        }
      } else {
        if (m_currentPressureLeft > Constants.Suction.kTargetPressure) {
          m_isMaximumPressureLeftReached = false;
        }
      } 

      if (!m_isMaximumPressureRightReached) {
        if (m_currentPressureRight > Constants.Suction.kMaximumPressure) {
          m_motorRight.set(Constants.Suction.kMaxSpeed);
        } else {
          m_motorRight.set(0);
          m_isMaximumPressureRightReached = true;
        }
      } else {
        if (m_currentPressureRight > Constants.Suction.kTargetPressure) {
          m_isMaximumPressureRightReached = false;
        }
      } 
      
      
    } else {
      if (m_isDisabling){
        m_motorTop.set(0);
        m_motorBottom.set(0);
        m_motorLeft.set(0);
        m_motorRight.set(0);
        m_solenoidBottom.set(true);
        m_solenoidTop.set(true);
        m_solenoidLeft.set(true);
        m_solenoidRight.set(true);
        m_isMaximumPressureBottomReached = false;
        m_isMaximumPressureTopReached = false;
        m_isMaximumPressureLeftReached = false;
        m_isMaximumPressureRightReached = false;
        m_isDisabling = false;
      } else {
        reset();
      }
    } 

    updateTelemetry();

    logSuction();
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

  public boolean isVacuumEnabledForCone() {
    return 
      m_currentPressureBottom <= Constants.Suction.kMinimumPressure && 
      m_currentPressureTop <= Constants.Suction.kMinimumPressure;
  }

  public boolean isVacuumEnabledForCube() {
    return 
      m_currentPressureLeft <= Constants.Suction.kTargetPressure && 
      m_currentPressureRight <= Constants.Suction.kTargetPressure;
  }

  public boolean isVacuumDisabled() {
    return 
      m_currentPressureBottom > Constants.Suction.kMinimumPressure && 
      m_currentPressureTop > Constants.Suction.kMinimumPressure && 
      m_currentPressureLeft > Constants.Suction.kMinimumPressure && 
      m_currentPressureRight > Constants.Suction.kMinimumPressure;
  }

  public void reset() {
    m_isEnabled = false;
    m_isDisabling = false;
    m_isMaximumPressureBottomReached = false;
    m_isMaximumPressureTopReached = false;
    m_isMaximumPressureLeftReached = false;
    m_isMaximumPressureRightReached = false;
    m_motorBottom.set(0);
    m_motorTop.set(0);
    m_motorLeft.set(0);
    m_motorRight.set(0);
    SmartDashboard.putBoolean("Suction/IsEnabled", m_isEnabled);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);

    builder.addDoubleProperty("Bottom/Motor/Speed", m_motorBottom::get, null);
    builder.addDoubleProperty("Top/Motor/Speed", m_motorTop::get, null);
    builder.addDoubleProperty("Left/Motor/Speed", m_motorLeft::get, null);
    builder.addDoubleProperty("Right/Motor/Speed", m_motorRight::get, null);
  }

  private void updateTelemetry() {
    SmartDashboard.putNumber("Suction/Pressure/Current/Bottom", m_currentPressureBottom);
    SmartDashboard.putNumber("Suction/Pressure/Current/Top", m_currentPressureTop);
    SmartDashboard.putNumber("Suction/Pressure/Current/Left", m_currentPressureLeft);
    SmartDashboard.putNumber("Suction/Pressure/Current/Right", m_currentPressureRight);
  }

  private void logSuction() {
    m_logSuctionBottomPressure.append(m_currentPressureBottom);
    m_logSuctionBottomOutput.append(m_motorBottom.getAppliedOutput());
    m_logSuctionBottomBusVoltage.append(m_motorBottom.getBusVoltage());
    m_logSuctionBottomCurrent.append(m_motorBottom.getOutputCurrent());
    
    m_logSuctionTopPressure.append(m_currentPressureTop);
    m_logSuctionTopOutput.append(m_motorTop.getAppliedOutput());
    m_logSuctionTopBusVoltage.append(m_motorTop.getBusVoltage());
    m_logSuctionTopCurrent.append(m_motorTop.getOutputCurrent());

    m_logSuctionLeftPressure.append(m_currentPressureLeft);
    m_logSuctionLeftOutput.append(m_motorLeft.getAppliedOutput());
    m_logSuctionLeftBusVoltage.append(m_motorLeft.getBusVoltage());
    m_logSuctionLeftCurrent.append(m_motorLeft.getOutputCurrent());

    m_logSuctionRightPressure.append(m_currentPressureRight);
    m_logSuctionRightOutput.append(m_motorRight.getAppliedOutput());
    m_logSuctionRightBusVoltage.append(m_motorRight.getBusVoltage());
    m_logSuctionRightCurrent.append(m_motorRight.getOutputCurrent());
  }
}
