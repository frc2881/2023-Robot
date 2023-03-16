// Copyright (c) 2023 FRC Team 2881 - The Lady Cans
//
// Open Source Software; you can modify and/or share it under the terms of BSD
// license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
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
    Charge
  }

  public static enum ColorPreset 
  {
    Cone(0x1A0F00),
    Cube(0x0D001A),
    Charge(0x33001A), 
    Pink(0x33001A),
    Black(0x000000);
  
    private int color;
    ColorPreset(int color) { this.color = color; }
    public int getColor() { return color; }
  }

  private final AddressableLED m_led;
  private final AddressableLEDBuffer m_ledBuffer;
  private boolean m_isBufferUpdated = false;

  public PrettyLights() {
    m_led = new AddressableLED(6);
    m_ledBuffer = new AddressableLEDBuffer(128);
    m_led.setLength(m_ledBuffer.getLength());
    m_led.setData(m_ledBuffer);
    m_led.start();
  }

  @Override
  public void periodic() {
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
}

