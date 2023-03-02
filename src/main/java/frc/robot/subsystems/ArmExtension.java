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

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmExtension extends SubsystemBase {
  private final CANSparkMax m_extensionMotor;
  private final SparkMaxPIDController m_extensionPID;
  private final RelativeEncoder m_extensionMotorEncoder;

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
    m_extensionMotor.setSmartCurrentLimit(20);
    m_extensionMotor.setSecondaryCurrentLimit(20, 0);

    m_extensionMotorEncoder = m_extensionMotor.getEncoder();
    m_extensionMotorEncoder.setPositionConversionFactor(Constants.Arm.kExtendRotationsToInches);
    
    m_extensionPID = m_extensionMotor.getPIDController();
    m_extensionPID.setFeedbackDevice(m_extensionMotorEncoder);
    m_extensionPID.setP(Constants.Arm.kExtensionP);
    m_extensionPID.setD(Constants.Arm.kExtensionD);
    m_extensionPID.setOutputRange(Constants.Arm.kExtensionMinOutput,
                                  Constants.Arm.kExtensionMaxOutput);

  }

  @Override
  public void periodic() {
    updateTelemetry();
  }

  /**
   * Extends or retracts the arm
   * @param speed positive value extends
   */
  public void run(double speed) { 
    m_extensionMotor.set(speed);
  }

  /*
   * Sets the Extension position to given value.
   */
  public void setDesiredPosition(double position, double speed) {
    m_extensionPID.setOutputRange(
      Constants.Arm.kExtensionMinOutput * speed,
      Constants.Arm.kExtensionMaxOutput * speed
    );
    m_extensionPID.setReference(position, CANSparkMax.ControlType.kPosition);
  }

  // In inches
  public Double getEncoderPosition() {
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
    m_extensionMotor.set(0);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);

    builder.addDoubleProperty("Extend/Position", m_extensionMotorEncoder::getPosition, null);
    builder.addDoubleProperty("Extend/Motor/Speed", m_extensionMotor::get, null);
    //builder.addBooleanProperty("Extend/IsSafe", this::isSafeToExtend, null);
  }

  private void updateTelemetry() {
    SmartDashboard.putNumber("Arm/Extend/Position", m_extensionMotorEncoder.getPosition());
  }
}
