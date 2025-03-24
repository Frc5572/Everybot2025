package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;
import edu.wpi.first.units.measure.Distance;

public interface ElevatorIO {

    @AutoLog
    public static class ElevatorInputs {
        Distance position;
    }

    public default void updateInputs(ElevatorInputs inputs) {}

    public void setPosition(double positon);

    public void setVoltage(double v);
}
