package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;
import org.littletonrobotics.junction.AutoLog;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;

/** Elevator IO interface */
public interface ElevatorIO {

    /** Elevator inputs */
    @AutoLog
    public static class ElevatorInputs {
        Voltage voltage = Volts.of(0);
        Distance position = Meters.of(0);
        Angle rotation = Rotations.of(0);
    }

    public default void updateInputs(ElevatorInputs inputs) {}

    public default void setPosition(double position) {}

    public default void setVoltage(double v) {}
}
