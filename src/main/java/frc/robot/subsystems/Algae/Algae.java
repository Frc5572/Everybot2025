package frc.robot.subsystems.algae;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Algae Subsystem
 */
public class Algae extends SubsystemBase {
    private AlgaeIO io;
    private AlgaeInputsAutoLogged inputs = new AlgaeInputsAutoLogged();



    /**
     * Algae subsystem
     *
     * @param io IO for the Algae
     */
    public Algae(AlgaeIO io) {
        this.io = io;
        io.updateInputs(inputs);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Algae", inputs);
    }

    /**
     * Set Motor power for the Algae intake/outake
     *
     * @param voltage voltage for the motor for the intake/outake
     */
    public void setAlgaeVoltage(double voltage) {
        Logger.recordOutput("Algae/Voltage", voltage);
        io.setAlgaeVoltage(voltage);
    }


    /**
     * Set motor power for the wrist
     *
     * @param voltage voltage for the motor for the wrist
     */
    public void setAlgaeWristVoltage(double voltage) {
        io.setAlgaeWristVolatage(voltage);
    }



    /**
     * Command for the intake set intake voltage
     *
     * @return Command
     */
    public Command runAlgaeIntake() {
        return Commands.runEnd(() -> setAlgaeVoltage(12), () -> setAlgaeVoltage(0));
    }

    /**
     * Command for the outake set outake voltage
     *
     * @return Command
     */
    public Command runAlgaeOuttake() {
        return Commands.runEnd(() -> setAlgaeVoltage(-12), () -> setAlgaeVoltage(0));
    }

    /**
     * Command for the Wrist to go down set motor voltage negative
     *
     * @return Command
     */
    public Command algaeWristDown() {
        return Commands.runEnd(() -> setAlgaeWristVoltage(-12), () -> setAlgaeWristVoltage(0));
    }

    /**
     * Command for the Wrist to go up set motor voltage positive
     *
     * @return Command
     */
    public Command algaeWristUp() {
        return Commands.runEnd(() -> setAlgaeWristVoltage(12), () -> setAlgaeWristVoltage(0));

    }
}
