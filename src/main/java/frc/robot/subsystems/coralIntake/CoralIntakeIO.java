package frc.robot.subsystems.coralIntake;

import org.littletonrobotics.junction.AutoLog;
import edu.wpi.first.units.measure.AngularVelocity;

public interface CoralIntakeIO {
    @AutoLog
    public static class CoralIntakeInputs {
        public AngularVelocity coralRPM;

    }

    public default void updateInputs(CoralIntakeInputs inputs) {}

    public default void setCoralVoltage(double Voltage) {}

}
