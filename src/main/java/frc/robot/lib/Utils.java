package frc.robot.lib;

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

}
