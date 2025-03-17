package frc.robot.subsystems.coralIntake;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;


public class CoralIntake extends SubsystemBase {
    private CoralIntakeIO io;
    private CoralIntakeInputsAutoLogged inputs = new CoralIntakeInputsAutoLogged();
    private CommandXboxController operator;

    private boolean pidEnabled = false;

    public CoralIntake(CoralIntakeIO io) {
        this.io = io;
        io.updateInputs(inputs);

        PIDController wristPIDController = new PIDController(Constants.CoralSubsystem.WristPID_KP,
            Constants.CoralSubsystem.WristPID_KI, Constants.CoralSubsystem.WristPID_KD);

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
        return Commands.runEnd(() -> setCoralVoltage(6), () -> setCoralVoltage(0));
    }

    public Command runCoralOuttake() {
        return Commands.runEnd(() -> setCoralVoltage(-6), () -> setCoralVoltage(0));
    }


}
