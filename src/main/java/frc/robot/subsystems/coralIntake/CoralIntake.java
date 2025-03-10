package frc.robot.subsystems.coralIntake;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class CoralIntake extends SubsystemBase {
    private CoralIntakeIO io;
    private CoralIntakeInputsAutoLogged inputs = new CoralIntakeInputsAutoLogged();

    public CoralIntake(CoralIntakeIO io) {
        this.io = io;
        io.updateInputs(inputs);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Coral", inputs);


    }

    public void setCoralVoltage(double voltage) {
        Logger.recordOutput("Coral/Voltage", voltage);
        io.setCoralVoltage(voltage);
    }

    public Command runCoralIntake() {
        return motorStartEndCommand(Constants.CoralSubsystemConstants.INTAKE_VOLTAGE);

    }

    public Command runCoralOuttake() {
        return motorStartEndCommand(Constants.CoralSubsystemConstants.OUTTAKE_VOLTAGE);

    }


}
