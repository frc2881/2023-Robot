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

public class ArmExtension extends SubsystemBase {
  private final CANSparkMax m_extensionMotor;
  private final SparkMaxPIDController m_extensionPID;
  private final RelativeEncoder m_extensionMotorEncoder;
  private final DoubleLogEntry m_logExtensionPosition;
  private final DoubleLogEntry m_logExtensionAppliedOutput;
  private final DoubleLogEntry m_logExtensionBusVoltage;
  private final DoubleLogEntry m_logExtensionOutputCurrent;

  public boolean m_isCube = false;

  private double m_v = (33.0 / Constants.Arm.kExtendRotationsToInches) * 60;
  private double m_a = (100.0 / Constants.Arm.kExtendVelocityConversion);

  public ArmExtension() {
    m_extensionMotor = new CANSparkMax(Constants.Arm.kExtensionMotorId, MotorType.kBrushless);
    m_extensionMotor.restoreFactoryDefaults();
    m_extensionMotor.setIdleMode(IdleMode.kBrake); 
    m_extensionMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
    m_extensionMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward,
                       (float)Constants.Arm.kExtendForwardLimit); 
    m_extensionMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
    m_extensionMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse,
                       (float)Constants.Arm.kExtendReverseLimit);
    m_extensionMotor.setSmartCurrentLimit(60);
    m_extensionMotor.setSecondaryCurrentLimit(60, 0);

    m_extensionMotorEncoder = m_extensionMotor.getEncoder();
    m_extensionMotorEncoder.setPositionConversionFactor(Constants.Arm.kExtendRotationsToInches);
    m_extensionMotorEncoder.setVelocityConversionFactor(Constants.Arm.kExtendVelocityConversion);
    
    
    m_extensionPID = m_extensionMotor.getPIDController();
    m_extensionPID.setSmartMotionMaxAccel(m_a, 0);
    m_extensionPID.setFeedbackDevice(m_extensionMotorEncoder);
    m_extensionPID.setP(Constants.Arm.kExtensionP);
    m_extensionPID.setD(Constants.Arm.kExtensionD);
    m_extensionPID.setOutputRange(Constants.Arm.kExtensionMinOutput,
                                  Constants.Arm.kExtensionMaxOutput);

    DataLog log = DataLogManager.getLog();
    m_logExtensionPosition = new DoubleLogEntry(log, "/Arm/Extension/Position");
    m_logExtensionAppliedOutput = new DoubleLogEntry(log, "/Arm/Extension/Output");
    m_logExtensionBusVoltage = new DoubleLogEntry(log, "/Arm/Extension/BusVoltage");
    m_logExtensionOutputCurrent = new DoubleLogEntry(log, "/Arm/Extension/Current");

  }

  @Override
  public void periodic() {
    updateTelemetry();
    logExtension();
  }

  /**
   * Extends or retracts the arm
   * @param speed positive value extends
   */
  public void run(double speed) { 
    m_extensionMotor.set(speed);
  }

  /**
   * Sets the arm extension to a position with a speed.
   * @param position
   * @param speed percentage
   */
  public void setDesiredPosition(double position, double speed) {
    speed *= m_v;
    m_extensionPID.setSmartMotionMaxVelocity(speed, 0);
    m_extensionPID.setReference(position, CANSparkMax.ControlType.kSmartMotion);
  }

  /**
   * Sets the arm extension to a position with a set speed.
   * @param position
   */
  public void setDesiredPosition(double position) {
    m_extensionPID.setSmartMotionMaxVelocity(m_v, 0);
    m_extensionPID.setReference(position, CANSparkMax.ControlType.kSmartMotion);
  }

  // In inches
  public double getEncoderPosition() {
    return m_extensionMotorEncoder.getPosition();
  }

  public void resetEncoder() {
    m_extensionMotorEncoder.setPosition(0);
  }

  public void enableSoftLimits(boolean enable){
    if (enable) {
      m_extensionMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
      m_extensionMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
    } else {
      m_extensionMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, false);
      m_extensionMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, false);
    }
  }

  public void setCube(boolean isCube) {
    m_isCube = isCube;
  }

  public void reset() {
    m_extensionMotor.set(0);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);

    builder.addDoubleProperty("Extend/Position", m_extensionMotorEncoder::getPosition, null);
    builder.addDoubleProperty("Extend/Motor/Speed", m_extensionMotor::get, null);
  }

  private void updateTelemetry() {
    SmartDashboard.putNumber("Arm/Extend/Position", m_extensionMotorEncoder.getPosition());
    SmartDashboard.putNumber("Arm/Extend/Velocity", m_extensionMotorEncoder.getVelocity());
  }

  private void logExtension() {
    m_logExtensionPosition.append(m_extensionMotorEncoder.getPosition());
    m_logExtensionAppliedOutput.append(m_extensionMotor.getAppliedOutput());
    m_logExtensionBusVoltage.append(m_extensionMotor.getBusVoltage());
    m_logExtensionOutputCurrent.append(m_extensionMotor.getOutputCurrent());
  }
}
