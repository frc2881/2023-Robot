// Copyright (c) 2023 FRC Team 2881 - The Lady Cans
//
// Open Source Software; you can modify and/or share it under the terms of BSD
// license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmTilt extends SubsystemBase {
  private final CANSparkMax m_tiltMotor;
  private final RelativeEncoder m_tiltMotorEncoder;
  private final SparkMaxPIDController m_tiltPID;
  private final DoubleLogEntry m_logTiltPosition;
  private final DoubleLogEntry m_logTiltAppliedOutput;
  private final DoubleLogEntry m_logTiltBusVoltage;
  private final DoubleLogEntry m_logTiltOutputCurrent;

  public ArmTilt() {

    m_tiltMotor = new CANSparkMax(Constants.Arm.kTiltMotorId, MotorType.kBrushless);
    m_tiltMotor.restoreFactoryDefaults();
    m_tiltMotor.setIdleMode(IdleMode.kBrake);
    m_tiltMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
    m_tiltMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward,
                       (float)Constants.Arm.kTiltForwardLimit);
    m_tiltMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
    m_tiltMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse,
                       (float)Constants.Arm.kTiltReverseLimit);
    m_tiltMotor.setSmartCurrentLimit(60);
    m_tiltMotor.setSecondaryCurrentLimit(60, 0);

    m_tiltMotorEncoder = m_tiltMotor.getEncoder();
    m_tiltMotorEncoder.setPositionConversionFactor(Constants.Arm.kTiltRotationsToInches);

    m_tiltPID = m_tiltMotor.getPIDController();
    m_tiltPID.setP(Constants.Arm.kTiltP);
    m_tiltPID.setOutputRange(Constants.Arm.kTiltMinOutput,
                             Constants.Arm.kTiltMaxOutput);

    DataLog log = DataLogManager.getLog();
    m_logTiltPosition = new DoubleLogEntry(log, "/armTilt/position");
    m_logTiltAppliedOutput = new DoubleLogEntry(log, "/armTilt/output");
    m_logTiltBusVoltage = new DoubleLogEntry(log, "/armTilt/busVoltage");
    m_logTiltOutputCurrent = new DoubleLogEntry(log, "/armTilt/current");

  }

  @Override
  public void periodic() {
    updateTelemetry();
    logTilt();
  }

  /**
   * Tilts the arm.
   * @param speed positive value goes up.
   */
  public void run(double speed) {
    m_tiltMotor.set(speed);
  }

  /*
   * Sets the Tilt position to given value
   */
  public void setDesiredPosition(double position, double speed) {
    m_tiltPID.setOutputRange(
      Constants.Arm.kTiltMinOutput * speed,
      Constants.Arm.kTiltMaxOutput * speed
    );
    m_tiltPID.setReference(position, CANSparkMax.ControlType.kPosition);
  }


  // In inches
  public Double getEncoderPosition() {
    return m_tiltMotorEncoder.getPosition();
  }

  public void resetEncoder() {
    m_tiltMotorEncoder.setPosition(0);
  }


  public void enableSoftLimits(boolean enable){
    if (enable) {
      m_tiltMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
      m_tiltMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
    } else {
      m_tiltMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, false);
      m_tiltMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, false);
    }
  }


  /*public boolean isSafeToExtend() {
    double tilt = m_tiltMotorEncoder.getPosition();
    m_isExtendSafe = tilt < Constants.Arm.kMinSafeTilt;
    return m_isExtendSafe;
  }

  public boolean isSafeToTilt() {
    double extensionPosition = m_extensionMotorEncoder.getPosition();
    double tiltPosition = m_tiltMotorEncoder.getPosition();
    if (tiltPosition > Constants.Arm.kMinSafeTilt) {
      m_isTiltSafe = true;
    } else {
      m_isTiltSafe = extensionPosition > 0;
    }
    return m_isTiltSafe;
  }*/

  public void reset() {
    m_tiltMotor.set(0);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);

    builder.addDoubleProperty("Tilt/Position", m_tiltMotorEncoder::getPosition, null);
    builder.addDoubleProperty("Tilt/Motor/Speed", m_tiltMotor::get, null);
    //builder.addBooleanProperty("Tilt/IsSafe", this::isSafeToTilt, null);

  }

  private void updateTelemetry() {
    SmartDashboard.putNumber("Arm/Tilt/Position", m_tiltMotorEncoder.getPosition());
  }

  public void logTilt() {
    m_logTiltPosition.append(m_tiltMotorEncoder.getPosition());
    m_logTiltAppliedOutput.append(m_tiltMotor.getAppliedOutput());
    m_logTiltBusVoltage.append(m_tiltMotor.getBusVoltage());
    m_logTiltOutputCurrent.append(m_tiltMotor.getOutputCurrent());
  }
}
