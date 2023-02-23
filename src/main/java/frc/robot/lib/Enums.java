package frc.robot.lib;

public final class Enums {
    
    public static enum RobotMode 
    {
      DISABLED("DISABLED"), 
      AUTO("AUTO"), 
      TELEOP("TELEOP"), 
      TEST("TEST");
    
      private String mode;
      RobotMode(String mode) { this.mode = mode; }
      public String getMode() { return mode; }
    }
  
    public static enum RobotStatus 
    {
      DISABLED("DISABLED"), 
      ENABLED("ENABLED"),
      ESTOPPED("ESTOPPED");
    
      private String status;
      RobotStatus(String status) { this.status = status; }
      public String gertStatus() { return status; }
    }

    public static enum SwerveDriveMode {
        FIELD_CENTRIC, 
        ROBOT_CENTRIC
    }

}
