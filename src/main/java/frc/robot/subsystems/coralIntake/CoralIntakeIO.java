package frc.robot.subsystems.coralIntake;

import org.littletonrobotics.junction.AutoLog;

public interface CoralIntakeIO {
    @AutoLog
    public static class CoralIntakeInputs {
        public double wristMotorVoltage;
        public double wristMotorAmp;

    }

    public default void updateInputs(CoralIntakeInputs inputs) {}

    public default void setCoralVoltage(double voltage) {}

    public default void setWristVolatage(double voltage) {}

}
