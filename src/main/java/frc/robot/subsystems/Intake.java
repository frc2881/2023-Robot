// Copyright (c) 2023 FRC Team 2881 - The Lady Cans
//
// Open Source Software; you can modify and/or share it under the terms of BSD
// license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  private final CANSparkMax m_rollers;
  private final CANSparkMax m_arm;
  private ColorSensorV3 m_colorSensor;
  private final ColorMatch m_colorMatcher;
  private boolean m_pieceIsCube;
  private boolean m_pieceIsCone;
  
  public Intake() {
    m_rollers = new CANSparkMax(Constants.Intake.kIntakeRollersCANId, MotorType.kBrushless);
    m_arm = new CANSparkMax(Constants.Intake.kIntakeArmCANId, MotorType.kBrushless);

    m_colorSensor = new ColorSensorV3(Port.kMXP);

    m_colorMatcher = new ColorMatch();
    m_colorMatcher.addColorMatch(Constants.Intake.kConeColor);
    m_colorMatcher.addColorMatch(Constants.Intake.kCubeColor);
    m_colorMatcher.setConfidenceThreshold(0.95);
  }

  /**
   * Runs the rollers
   * 
   * @param speed positive value runs the rollers forward
   */
  public void runRollers(double speed){
    m_rollers.set(speed);
  }

  /**
   * Moves the arm in or out
   * 
   * @param speed positive value extends
   */
  public void moveArm(double speed){
    m_arm.set(speed);
  }

  /**
   * @return is the game piece a cube
   */
  public boolean isCube() {
    return m_pieceIsCube;
  }

  /**
   * @return is the game piece a cone
   */
  public boolean isCone() {
    return m_pieceIsCone;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Color detectedColor;
    int distance;

    if(m_colorSensor.isConnected()) {
      detectedColor = m_colorSensor.getColor();
      distance = m_colorSensor.getProximity();
    } else {
      detectedColor = new Color(0, 0, 0);
      distance = 0;
    }
    ColorMatchResult match = m_colorMatcher.matchColor(detectedColor);

    if((match != null) && (match.color == Constants.Intake.kConeColor) &&
       (distance > Constants.Intake.kDistance)) {
      m_pieceIsCone = true;
    } else {
      m_pieceIsCone = false;
    }

    if((match != null) && (match.color == Constants.Intake.kCubeColor) &&
       (distance > Constants.Intake.kDistance)) {
      m_pieceIsCube = true;
    } else {
      m_pieceIsCube = false;
    }

    if(m_colorSensor.isConnected() && m_colorSensor.hasReset()) {
      m_colorSensor = new ColorSensorV3(Port.kMXP);
    }

    // Use for finding the Color values for cone/cube.
    Color color = m_colorSensor.getColor();

    SmartDashboard.putNumber("Red", color.red);
    SmartDashboard.putNumber("Green", color.green);
    SmartDashboard.putNumber("Blue", color.blue);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    if(m_colorSensor.isConnected()) {
      builder.addDoubleProperty("Sensor Distance", () -> m_colorSensor.getProximity(), null);
    }
    builder.addBooleanProperty("Cone", () -> isCube(), null);
    builder.addBooleanProperty("Cube", () -> isCone(), null);

  }
}
