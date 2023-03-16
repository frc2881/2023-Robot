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
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Clamps extends SubsystemBase {
  private final CANSparkMax m_left;
  private final CANSparkMax m_right;

  private final DoubleLogEntry m_logLeftClampPosition;
  private final DoubleLogEntry m_logLeftClampAppliedOutput;
  private final DoubleLogEntry m_logLeftClampBusVoltage;
  private final DoubleLogEntry m_logLeftClampOutputCurrent;

  private final DoubleLogEntry m_logRightClampPosition;
  private final DoubleLogEntry m_logRightClampAppliedOutput;
  private final DoubleLogEntry m_logRightClampBusVoltage;
  private final DoubleLogEntry m_logRightClampOutputCurrent;
  
  public Clamps() {
    m_left = new CANSparkMax(Constants.Clamps.kLeftClampCANId, MotorType.kBrushless);
    m_left.setIdleMode(IdleMode.kBrake);
    m_left.setSmartCurrentLimit(Constants.Clamps.kCurrentLimit);
    
    m_left.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
    m_left.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward,
                       Constants.Clamps.kReverseSoftLimit); 

    m_right = new CANSparkMax(Constants.Clamps.kRightClampCANId, MotorType.kBrushless);
    m_right.setIdleMode(IdleMode.kBrake);
    m_right.setSmartCurrentLimit(Constants.Clamps.kCurrentLimit);
    
    m_right.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
    m_right.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse,
                       Constants.Clamps.kReverseSoftLimit);
                       
    DataLog log = DataLogManager.getLog();
    
    m_logLeftClampPosition = new DoubleLogEntry(log, "/clamps/left/position");
    m_logLeftClampAppliedOutput = new DoubleLogEntry(log, "/clamps/left/output");
    m_logLeftClampBusVoltage = new DoubleLogEntry(log, "/clamps/left/busVoltage");
    m_logLeftClampOutputCurrent = new DoubleLogEntry(log, "/clamps/left/current");

    m_logRightClampPosition = new DoubleLogEntry(log, "/clamps/right/position");
    m_logRightClampAppliedOutput = new DoubleLogEntry(log, "/clamps/right/output");
    m_logRightClampBusVoltage = new DoubleLogEntry(log, "/clamps/right/busVoltage");
    m_logRightClampOutputCurrent = new DoubleLogEntry(log, "/clamps/right/current");
  }

  @Override
  public void periodic() {
    updateTelemetry();
    logClamps();
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
  }

  public void attachLeft(double speed) {
    m_left.set(speed);
  }
  public void attachRight(double speed) {
    m_right.set(speed);
  }

  public void updateTelemetry() {}

  private void logClamps() {
    m_logLeftClampPosition.append(m_left.getEncoder().getPosition());
    m_logLeftClampAppliedOutput.append(m_left.getAppliedOutput());
    m_logLeftClampBusVoltage.append(m_left.getBusVoltage());
    m_logLeftClampOutputCurrent.append(m_left.getOutputCurrent());

    m_logRightClampPosition.append(m_right.getEncoder().getPosition());
    m_logRightClampAppliedOutput.append(m_right.getAppliedOutput());
    m_logRightClampBusVoltage.append(m_right.getBusVoltage());
    m_logRightClampOutputCurrent.append(m_right.getOutputCurrent());
  }
}
