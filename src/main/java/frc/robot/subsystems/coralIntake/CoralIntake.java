package frc.robot.subsystems.coralIntake;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralIntake extends SubsystemBase {
    private CoralIntakeIO io;
    private CoralIntakeInputsAutoLogged inputs = new CoralIntakeInputsAutoLogged();

    public Coralintake(CoralIntakeIO io){
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



}
