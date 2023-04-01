// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib;

import edu.wpi.first.math.geometry.Pose2d;

/** Add your docs here. */
public class Node {
  public Pose2d pose;
  public NodeType nodeType;
  public int slot;

  public Node(Pose2d pose, NodeType nodeType, int slot) {
    this.pose = pose;
    this.nodeType = nodeType;
    this.slot = slot;

  }

  public static enum NodeType
  {
    CONE("CONE"), 
    CUBE("CUBE");
  
    private String nodeType;
    NodeType(String nodeType) { this.nodeType = nodeType; }
    public String getNodeType() { return nodeType; }
  }

}
