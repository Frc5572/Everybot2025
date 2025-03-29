package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/** Elevator subsystem */
public class Elevator extends SubsystemBase {

    ElevatorIO io;
    ElevatorInputsAutoLogged inputs = new ElevatorInputsAutoLogged();

    /** Elevator initlizer */
    public Elevator(ElevatorIO io) {
        this.io = io;
        io.updateInputs(inputs);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);
    }


    public Command p0() {
        return moveTo(() -> Constants.Elevator.P0);
    }

    public Command p1() {
        return moveTo(() -> Constants.Elevator.P1);
    }

    public Command p2() {
        return moveTo(() -> Constants.Elevator.P2);
    }

    public Command p3() {
        return moveTo(() -> Constants.Elevator.P3);
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

    public Command setVoltage(DoubleSupplier v) {
        return runEnd(() -> io.setVoltage(v.getAsDouble()), (() -> io.setVoltage(0)));
    }
}
