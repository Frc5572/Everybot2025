package frc.robot.subsystems.Algae;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Algae extends SubsystemBase {
    private AlgaeIO io;
    private AlgaeInputsAutoLogged inputs = new AlgaeInputsAutoLogged();



    public Algae(AlgaeIO io) {
        this.io = io;
        io.updateInputs(inputs);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Algae", inputs);
    }

    public void setAlgaeVoltage(double voltage) {
        Logger.recordOutput("Algae/Voltage", voltage);
        io.setAlgaeVoltage(voltage);
    }


    public void setAlgaeWristVoltage(double voltage) {
        io.setAlgaeWristVolatage(voltage);
    }



    public Command runAlgaeIntake() {
        return Commands.runEnd(() -> setAlgaeVoltage(12), () -> setAlgaeVoltage(0));
    }

    public Command runAlgaeOuttake() {
        return Commands.runEnd(() -> setAlgaeVoltage(-12), () -> setAlgaeVoltage(0));
    }

    public Command algaeWristDown() {
        return Commands.runEnd(() -> setAlgaeWristVoltage(-12), () -> setAlgaeWristVoltage(0));
    }

    public Command algaeWristUp() {
        return Commands.runEnd(() -> setAlgaeWristVoltage(12), () -> setAlgaeWristVoltage(0));

    }
}
