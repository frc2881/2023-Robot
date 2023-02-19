// Copyright (c) 2023 FRC Team 2881 - The Lady Cans
//
// Open Source Software; you can modify and/or share it under the terms of BSD
// license file in the root directory of this project.

package frc.robot.lib;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAnalogSensor;
import com.revrobotics.SparkMaxAnalogSensor.Mode;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants;

public class SwerveModule {
  private final CANSparkMax m_drivingSparkMax;
  private final CANSparkMax m_turningSparkMax;

  private final RelativeEncoder m_drivingEncoder;
  private final RelativeEncoder m_turningEncoder;
  private final SparkMaxAnalogSensor m_turningAnalogSensor;

  private final SparkMaxPIDController m_drivingPIDController;
  private final SparkMaxPIDController m_turningPIDController;

  private double m_chassisAngularOffset = 0;
  private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

  private double m_drivingCANId;
  /**
   * Constructs a MAXSwerveModule and configures the driving and turning motor,
   * encoder, and PID controller. This configuration is specific to the REV
   * MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
   * Encoder.
   */
  public SwerveModule(int drivingCANId, int turningCANId, double chassisAngularOffset) {
    m_drivingSparkMax = new CANSparkMax(drivingCANId, MotorType.kBrushless);
    m_turningSparkMax = new CANSparkMax(turningCANId, MotorType.kBrushless);

    m_drivingCANId = drivingCANId;

    // Factory reset, so we get the SPARKS MAX to a known state before configuring
    // them. This is useful in case a SPARK MAX is swapped out.
    m_drivingSparkMax.restoreFactoryDefaults();
    m_turningSparkMax.restoreFactoryDefaults();

    // SDS Module is inverted relative to the MAXSwerve
    m_drivingSparkMax.setInverted(true);;
    m_turningSparkMax.setInverted(true);;

    // Setup encoders and PID controllers for the driving and turning SPARKS MAX.
    m_drivingEncoder = m_drivingSparkMax.getEncoder();
    m_turningEncoder = m_turningSparkMax.getEncoder();
    m_turningAnalogSensor = m_turningSparkMax.getAnalog(Mode.kAbsolute);
    m_drivingPIDController = m_drivingSparkMax.getPIDController();
    m_turningPIDController = m_turningSparkMax.getPIDController();
    m_drivingPIDController.setFeedbackDevice(m_drivingEncoder);
    m_turningPIDController.setFeedbackDevice(m_turningEncoder);

    // Apply position and velocity conversion factors for the driving encoder. The
    // native units for position and velocity are rotations and RPM, respectively,
    // but we want meters and meters per second to use with WPILib's swerve APIs.
    m_drivingEncoder.setPositionConversionFactor(Constants.SwerveModule.kDrivingEncoderPositionFactor);
    m_drivingEncoder.setVelocityConversionFactor(Constants.SwerveModule.kDrivingEncoderVelocityFactor);

    // Apply position and velocity conversion factors for the turning encoder. We
    // want these in radians and radians per second to use with WPILib's swerve
    // APIs.
    m_turningEncoder.setPositionConversionFactor(Constants.SwerveModule.kTurningEncoderPositionFactor);
    m_turningEncoder.setVelocityConversionFactor(Constants.SwerveModule.kTurningEncoderVelocityFactor);
    m_turningAnalogSensor.setPositionConversionFactor(Constants.SwerveModule.kTurningAnalogPositionFactor);
    m_turningAnalogSensor.setVelocityConversionFactor(Constants.SwerveModule.kTurningAnalogVelocityFactor);

    // Enable PID wrap around for the turning motor. This will allow the PID
    // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
    // to 10 degrees will go through 0 rather than the other direction which is a
    // longer route.
    m_turningPIDController.setPositionPIDWrappingEnabled(true);
    m_turningPIDController.setPositionPIDWrappingMinInput(Constants.SwerveModule.kTurningEncoderPositionPIDMinInput);
    m_turningPIDController.setPositionPIDWrappingMaxInput(Constants.SwerveModule.kTurningEncoderPositionPIDMaxInput);

    // Set the PID gains for the driving motor. Note these are example gains, and you
    // may need to tune them for your own robot!
    m_drivingPIDController.setP(Constants.SwerveModule.kDrivingP);
    m_drivingPIDController.setI(Constants.SwerveModule.kDrivingI);
    m_drivingPIDController.setD(Constants.SwerveModule.kDrivingD);
    m_drivingPIDController.setFF(Constants.SwerveModule.kDrivingFF);
    m_drivingPIDController.setOutputRange(Constants.SwerveModule.kDrivingMinOutput,
      Constants.SwerveModule.kDrivingMaxOutput);

    // Set the PID gains for the turning motor. Note these are example gains, and you
    // may need to tune them for your own robot!
    m_turningPIDController.setP(Constants.SwerveModule.kTurningP);
    m_turningPIDController.setI(Constants.SwerveModule.kTurningI);
    m_turningPIDController.setD(Constants.SwerveModule.kTurningD);
    m_turningPIDController.setFF(Constants.SwerveModule.kTurningFF);
    m_turningPIDController.setOutputRange(Constants.SwerveModule.kTurningMinOutput,
        Constants.SwerveModule.kTurningMaxOutput);

    m_drivingSparkMax.setIdleMode(Constants.SwerveModule.kDrivingMotorIdleMode);
    m_turningSparkMax.setIdleMode(Constants.SwerveModule.kTurningMotorIdleMode);
    m_drivingSparkMax.setSmartCurrentLimit(Constants.SwerveModule.kDrivingMotorCurrentLimit);
    m_turningSparkMax.setSmartCurrentLimit(Constants.SwerveModule.kTurningMotorCurrentLimit);

    // Save the SPARK MAX configurations. If a SPARK MAX browns out during
    // operation, it will maintain the above configurations.
    m_drivingSparkMax.burnFlash();
    m_turningSparkMax.burnFlash();

    // This allows time for the absolute position to be sent by the CANcoder (we know this isn't the best solution, we'll fix it later)
    Timer.delay(1);

    // This is commented out because it is already being calculated by the CANcoder
    // m_chassisAngularOffset = chassisAngularOffset; 
    
    double initialAngle = m_turningAnalogSensor.getPosition() - chassisAngularOffset;
    if(initialAngle > 0.1 || initialAngle < 0.0){
      DataLog.log("ERROR - Initial Angle Bad: " + initialAngle + " Position from Analog: " + m_turningAnalogSensor.getPosition() + " CANId: " + turningCANId);
      initialAngle = 0.0;
    }
    m_desiredState.angle = new Rotation2d(initialAngle);
    m_drivingEncoder.setPosition(0.0);
    m_turningEncoder.setPosition(initialAngle);
    
  }


  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModuleState(m_drivingEncoder.getVelocity(),
        new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModulePosition(
        m_drivingEncoder.getPosition(),
        new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Apply chassis angular offset to the desired state.
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset));

    // Optimize the reference state to avoid spinning further than 90 degrees.
    SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(correctedDesiredState,
        new Rotation2d(m_turningEncoder.getPosition()));

    // Command driving and turning SPARKS MAX towards their respective setpoints.
    m_drivingPIDController.setP(SmartDashboard.getNumber("P", Constants.SwerveModule.kDrivingP));
    m_drivingPIDController.setD(SmartDashboard.getNumber("D", Constants.SwerveModule.kDrivingD));
    m_drivingPIDController.setReference(optimizedDesiredState.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity);
    m_turningPIDController.setReference(optimizedDesiredState.angle.getRadians(), CANSparkMax.ControlType.kPosition);

    if(m_drivingCANId == 1 && DriverStation.isAutonomous()){
      System.out.println(desiredState.angle.getRadians() + " " + correctedDesiredState.angle.getRadians() + " " + m_turningEncoder.getPosition());
    }

    m_desiredState = desiredState;

  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    m_drivingEncoder.setPosition(0.0);
  }

  public double getSteeringRelativePosition(){
   return m_turningEncoder.getPosition();
  }

  public double getSteeringAbsolutePosition(){
    return m_turningAnalogSensor.getPosition();
  }

  public double getDrivingRelativePosition(){
    return m_drivingEncoder.getPosition();
  }

  public double getDrivingVelocity(){
    return m_drivingEncoder.getVelocity();
  }
}