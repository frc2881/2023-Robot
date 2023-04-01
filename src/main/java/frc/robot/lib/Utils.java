package frc.robot.lib;

import edu.wpi.first.math.geometry.Pose2d;

public final class Utils {
    
    public static double applyDeadband(double input, double deadband) {
        if (Math.abs(input) < deadband)
            return 0.0;

        return Math.copySign((Math.abs(input) - deadband) / (1.0 - deadband), input);
    }

     /** Converts volts to PSI per the REV Analog Pressure Sensor datasheet. */
    public static double voltsToPsi(double sensorVoltage, double supplyVoltage) {
        return 250 * (sensorVoltage / supplyVoltage) - 25;
    }

    public static boolean isValueBetween(double value, double minValue, double maxValue) {
        return value >= minValue && value <= maxValue;
    }

    public static boolean isPoseInBounds(Pose2d value, Pose2d minPose, Pose2d maxPose) {
        return 
            isValueBetween(value.getX(), minPose.getX(), maxPose.getX()) && 
            isValueBetween(value.getY(), minPose.getY(), maxPose.getY());
    }

}
