package frc.lib.math;

/**
 * Mathematical conversions for swerve calculations
 */
public class Conversions {

    public static double rotationPerSecondToMetersPerSecond(double wheelRPS, double circumference) {
        double wheelMPS = wheelRPS * circumference;
        return wheelMPS;
    }

    public static double metersPerSecondToRotationPerSecond(double wheelMPS, double circumference) {
        double wheelRPS = wheelMPS / circumference;
        return wheelRPS;
    }

    public static double rotationsToMeters(double wheelRotations, double circumference) {
        double wheelMeters = wheelRotations * circumference;
        return wheelMeters;
    }


    /**
     * Normalize angle to between 0 to 360
     *
     * @param goal initial angle
     * @return normalized angle
     */
    public static double reduceTo0_360(double goal) {
        return goal % 360;
    }

}
