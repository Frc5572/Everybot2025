package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;
import edu.wpi.first.units.measure.Distance;

/** Elevator IO interface */
public interface ElevatorIO {

    /** Elevator inputs */
    @AutoLog
    public static class ElevatorInputs {
        Distance position;
    }

    public default void updateInputs(ElevatorInputs inputs) {}

    public default void setPosition(double positon) {}

    public default void setVoltage(double v) {}
}
