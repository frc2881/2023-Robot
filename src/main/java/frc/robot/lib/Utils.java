package frc.robot.lib;

public final class Utils {
    
    public static double applyDeadband(double input, double deadband) {
        return (Math.abs(input) < deadband) ? 0.0 : Math.copySign((Math.abs(input) - deadband) / 0.9, input);
    }

     /** Converts volts to PSI per the REV Analog Pressure Sensor datasheet. */
    public static double voltsToPsi(double sensorVoltage, double supplyVoltage) {
        return 250 * (sensorVoltage / supplyVoltage) - 25;
    }

}
