package frc.robot.subsystems.coral;

import org.littletonrobotics.junction.AutoLog;

/**
 * Coral Intake IO
 */
public interface CoralIntakeIO {
    /** Coral Intake inputs */
    @AutoLog
    public static class CoralIntakeInputs {
        public double wristMotorVoltage;
        public double wristMotorAmp;
        public double wristAngle;

    }

    public default void updateInputs(CoralIntakeInputs inputs) {}

    public default void setCoralVoltage(double voltage) {}

    public default void setWristVolatage(double voltage) {}

    public default void setWristSetPoint(double setPoint) {}

    public default double getWristRotations() {
        return 0;
    }
}
