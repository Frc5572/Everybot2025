package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {

    ElevatorIO io;
    ElevatorInputsAutoLogged inputs = new ElevatorInputsAutoLogged();

    Elevator(ElevatorIO io) {
        this.io = io;
        io.updateInputs(inputs);
    }

    @Override
    public void periodic() {
        Logger.processInputs("Elevator", inputs);
    }

    /**
     * sets height of elevator
     *
     * @param height desired height of elevator
     * @return elevator height change
     *
     */
    public Command moveTo(Supplier<Distance> height) {
        return runOnce(() -> {
            Logger.recordOutput("Elevator/targetHeight", height.get().in(Meters));
            io.setPosition(height.get().in(Meters));
        }).andThen(Commands
            .waitUntil(() -> Math.abs(inputs.position.in(Inches) - height.get().in(Inches)) < 1));
    }


}
