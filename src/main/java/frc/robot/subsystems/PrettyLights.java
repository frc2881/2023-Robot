// Copyright (c) 2023 FRC Team 2881 - The Lady Cans
//
// Open Source Software; you can modify and/or share it under the terms of BSD
// license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Random;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PrettyLights extends SubsystemBase {

  public static enum PanelLocation {
    Front,
    Rear,
    Both
  }

  public static enum Pattern {
    None,
    Heart,
    Cone,
    Cube,
    Charge,
    Smile
  }

  public static enum ColorPreset 
  {
    Cone(0x1A0F00),
    Cube(0x0D001A),
    Charge(0x33001A), 
    Pink(0x33001A),
    Blue(0x000011),
    Black(0x000000),
    White(0x111111),
    Green(0x001100);
  
    private int color;
    ColorPreset(int color) { this.color = color; }
    public int getColor() { return color; }
  }

  private class AnimationFrame {
    private final int[] m_shape;
    private final double m_minimumDuration;
    private final double m_maximumDuration;

    public AnimationFrame(int[] shape, double minimumDuration, double maximumDuration){
      m_shape = shape;
      m_minimumDuration = minimumDuration;
      m_maximumDuration = maximumDuration;
    }
    public int[] getShape() {
      return m_shape;
    }
    public double getDuration() {
      return m_minimumDuration + Math.random() * (m_maximumDuration - m_minimumDuration);
    }
  }
  private final AddressableLED m_led;
  private final AddressableLEDBuffer m_ledBuffer;
  private boolean m_isBufferUpdated = false;
  private Pattern m_currentPattern = Pattern.None;
  private Random m_random = new Random();
  private double m_lastTimeStamp = Timer.getFPGATimestamp();
  private int m_animationFrameIndex = 0;
  private double m_animationFrameTimer = 0.0;

  public PrettyLights() {
    m_led = new AddressableLED(6);
    m_ledBuffer = new AddressableLEDBuffer(128);
    m_led.setLength(m_ledBuffer.getLength());
    m_led.setData(m_ledBuffer);
    m_led.start();
  }


  private void beginAnimationFrame(int index){
    m_animationFrameIndex = index;
    if (index >= m_currentAnimation.length){
      m_animationFrameIndex = 0;
    }
    m_animationFrameTimer = m_currentAnimation[m_animationFrameIndex].getDuration();
  }
  /* 
  @Override
  public void periodic() {
    // howdy, this runs an animation when the robot's disabled :')
    if(RobotState.isDisabled()){
      double currentTimeStamp = Timer.getFPGATimestamp();
      double elapsedTime = currentTimeStamp - m_lastTimeStamp;
      m_lastTimeStamp = currentTimeStamp;
      if (m_currentPattern == Pattern.Smile) {
        m_animationFrameTimer = m_animationFrameTimer - elapsedTime;
        if (m_animationFrameTimer <= 0) {
          beginAnimationFrame(m_animationFrameIndex + 1);
          setShape(m_currentAnimation[m_animationFrameIndex].getShape(), PanelLocation.Front);
          setShape(m_currentAnimation[m_animationFrameIndex].getShape(), PanelLocation.Rear);
        }
      }
    }

    if (m_isBufferUpdated) {
      m_led.setData(m_ledBuffer);
      m_isBufferUpdated = false;
    }
  }*/

  @Override
  public void periodic() {
    // Twinkle
    if(RobotState.isDisabled()){
      double currentTimeStamp = Timer.getFPGATimestamp();

      if((currentTimeStamp - m_lastTimeStamp) > 0.1){
        if(m_currentPattern == Pattern.Heart){
          int[] shape = m_shapeHeart.clone();

          for (int i = 0; i < 5; i += 1){
            int color = 0x111111; // Gold: 0xFFD700, Light Pink: 0x66335A
            int position = m_random.nextInt(64);

            if(shape[position] == 0x000000){ //0x000000
              shape[position] = color;
            }
            
          }

          setShape(shape, PanelLocation.Front);
          setShape(shape, PanelLocation.Rear);

          
        }
        m_lastTimeStamp = currentTimeStamp;
    }
  }
    if (m_isBufferUpdated) {
      m_led.setData(m_ledBuffer);
      m_isBufferUpdated = false;
    }
  }

 
  private void setShape(int[] shape, PanelLocation panel) {
    for (int i = 0; i < 64; i += 1) {
      int position = panel == PanelLocation.Rear ? m_panelRear[i] + 64 : m_panelFront[i];
      int color = shape[i];
      m_ledBuffer.setLED(position, new Color(color >> 16, (color >> 8) & 255, color & 255));
    }
    m_isBufferUpdated = true;
  }

  public void setPattern(Pattern pattern, PanelLocation panel) {
    if (pattern == m_currentPattern) { return; }

    int[] shape = m_shapeBlank;
    switch (pattern) {
      case Heart:
        shape = m_shapeHeart;
        break;
      case Cone:
        shape = m_shapeCone;
        break;
      case Cube:
        shape = m_shapeCube;
        break;
      case Charge:
        shape = m_shapeCharge;
        break;
      case Smile:
        shape = m_shapeFace;
        beginAnimationFrame(0);
        break;
      case None:
        shape = m_shapeBlank;
        break;
    }
    
    if (panel == PanelLocation.Both) {
      setShape(shape, PanelLocation.Front);
      setShape(shape, PanelLocation.Rear);
    }
    else {
      setShape(shape, panel);
    }

    m_currentPattern = pattern;

    SmartDashboard.putString("Lights/Pattern", pattern.toString());
  }

  private final int[] m_panelFront = {
    0, 15, 16, 31, 32, 47, 48, 63,
    1, 14, 17, 30, 33, 46, 49, 62,
    2, 13, 18, 29, 34, 45, 50, 61,
    3, 12, 19, 28, 35, 44, 51, 60,
    4, 11, 20, 27, 36, 43, 52, 59,
    5, 10, 21, 26, 37, 42, 53, 58,
    6,  9, 22, 25, 38, 41, 54, 57,
    7,  8, 23, 24, 39, 40, 55, 56
  };

  private final int[] m_panelRear = {
    56, 55, 40, 39, 24, 23,  8, 7,
    57, 54, 41, 38, 25, 22,  9, 6,
    58, 53, 42, 37, 26, 21, 10, 5,
    59, 52, 43, 36, 27, 20, 11, 4,
    60, 51, 44, 35, 28, 19, 12, 3,
    61, 50, 45, 34, 29, 18, 13, 2,
    62, 49, 46, 33, 30, 17, 14, 1,
    63, 48, 47, 32, 31, 16, 15, 0
  };

  private final int __ = ColorPreset.Black.getColor();
  private final int PK = ColorPreset.Pink.getColor();
  private final int CN = ColorPreset.Cone.getColor();
  private final int CB = ColorPreset.Cube.getColor();
  private final int CH = ColorPreset.Charge.getColor();
  private final int BL = ColorPreset.Blue.getColor();
  private final int WH = ColorPreset.White.getColor();
  private final int GR = ColorPreset.Green.getColor();

  private final int[] m_shapeBlank = {
    __, __, __, __, __, __, __, __,
    __, __, __, __, __, __, __, __,
    __, __, __, __, __, __, __, __,
    __, __, __, __, __, __, __, __,
    __, __, __, __, __, __, __, __,
    __, __, __, __, __, __, __, __,
    __, __, __, __, __, __, __, __,
    __, __, __, __, __, __, __, __
  };

  private final int[] m_shapeHeart = {
    __, __, __, __, __, __, __, __,
    __, PK, PK, __, __, PK, PK, __,
    PK, PK, PK, PK, PK, PK, PK, PK,
    PK, PK, PK, PK, PK, PK, PK, PK,
    PK, PK, PK, PK, PK, PK, PK, PK,
    __, PK, PK, PK, PK, PK, PK, __,
    __, __, PK, PK, PK, PK, __, __,
    __, __, __, PK, PK, __, __, __
  };

  private final int[] m_shapeCone = {
    __, __, __, CN, CN, __, __, __,
    __, __, __, CN, CN, __, __, __,
    __, __, __, CN, CN, __, __, __,
    __, __, CN, CN, CN, CN, __, __,
    __, __, CN, CN, CN, CN, __, __,
    __, __, CN, CN, CN, CN, __, __,
    CN, CN, CN, CN, CN, CN, CN, CN,
    CN, CN, CN, CN, CN, CN, CN, CN
  };

  private final int[] m_shapeCube = {
    __, __, __, __, __, __, __, __,
    __, __, CB, CB, CB, CB, __, __,
    __, CB, CB, CB, CB, CB, CB, __,
    __, CB, CB, CB, CB, CB, CB, __,
    __, CB, CB, CB, CB, CB, CB, __,
    __, CB, CB, CB, CB, CB, CB, __,
    __, __, CB, CB, CB, CB, __, __,
    __, __, __, __, __, __, __, __
  };

  private final int[] m_shapeCharge = {
    __, __, __, __, __, __, __, __,
    __, __, __, __, CH, __, __, __,
    __, __, __, CH, __, __, __, __,
    __, __, CH, CH, __, __, __, __,
    __, CH, CH, CH, CH, CH, CH, __,
    __, __, __, __, CH, CH, __, __,
    __, __, __, __, CH, __, __, __,
    __, __, __, CH, __, __, __, __
  };
  private final int[] m_shapeFace = {
    PK, PK, PK, __, __, PK, PK, PK,
    WH, GR, GR, __, __, GR, GR, WH,
    WH, GR, GR, __, __, GR, GR, WH,
    WH, GR, GR, __, __, GR, GR, WH,
    __, __, __, __, __, __, __, __,
    __, __, __, __, __, __, __, __,
    __, PK, __, __, __, __, PK, __,
    __, __, PK, PK, PK, PK, __, __
  };
  private final int[] m_shapeFaceBlink1 = {
    __, __, __, __, __, __, __, __,
    PK, PK, PK, __, __, PK, PK, PK,
    WH, GR, GR, __, __, GR, GR, WH,
    WH, GR, GR, __, __, GR, GR, WH,
    __, __, __, __, __, __, __, __,
    __, __, __, __, __, __, __, __,
    __, PK, __, __, __, __, PK, __,
    __, __, PK, PK, PK, PK, __, __
  };
  private final int[] m_shapeFaceBlink2 = {
    __, __, __, __, __, __, __, __,
    __, __, __, __, __, __, __, __,
    PK, PK, PK, __, __, PK, PK, PK,
    WH, GR, GR, __, __, GR, GR, WH,
    __, __, __, __, __, __, __, __,
    __, __, __, __, __, __, __, __,
    __, PK, __, __, __, __, PK, __,
    __, __, PK, PK, PK, PK, __, __
  };
  private final int[] m_shapeFaceBlink3 = {
    __, __, __, __, __, __, __, __,
    __, __, __, __, __, __, __, __,
    __, __, __, __, __, __, __, __,
    PK, PK, PK, __, __, PK, PK, PK,
    __, __, __, __, __, __, __, __,
    __, __, __, __, __, __, __, __,
    __, PK, __, __, __, __, PK, __,
    __, __, PK, PK, PK, PK, __, __
  };

  private AnimationFrame[] m_currentAnimation = {
    new AnimationFrame(m_shapeFace, 1, 7),
    new AnimationFrame(m_shapeFaceBlink1, 0.0833, 0.0833),
    new AnimationFrame(m_shapeFaceBlink2, 0.0833, 0.0833),
    new AnimationFrame(m_shapeFaceBlink3, 0.0833, 0.0833),
    new AnimationFrame(m_shapeFaceBlink2, 0.0833, 0.0833),
    new AnimationFrame(m_shapeFaceBlink1, 0.0833, 0.0833),

  };
}

