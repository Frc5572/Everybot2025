package frc.robot.subsystems.Algae;

import org.littletonrobotics.junction.AutoLog;

public interface AlgaeIO {
    @AutoLog
    public static class AlgaeInputs {
        public double algaewristMotorVoltage;
        public double algaewristMotorAmp;

    }

    public default void updateInputs(AlgaeInputs inputs) {}

    public default void setAlgaeVoltage(double voltage) {}

    public default void setAlgaeWristVolatage(double voltage) {}

}
