package frc.robot.subsystems.algae;

import org.littletonrobotics.junction.AutoLog;

/**
 * Algae IO
 */
public interface AlgaeIO {
    /**
     * Algae inputs
     */
    @AutoLog
    public static class AlgaeInputs {
        public double algaewristMotorVoltage;
        public double algaewristMotorAmp;

    }

    public default void updateInputs(AlgaeInputs inputs) {}

    public default void setAlgaeVoltage(double voltage) {}

    public default void setAlgaeWristVolatage(double voltage) {}

}
