package frc.robot.subsystems.coralIntake;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;


public class CoralIntake extends SubsystemBase {
    private CoralIntakeIO io;
    private CoralIntakeInputsAutoLogged inputs = new CoralIntakeInputsAutoLogged();
    private CommandXboxController operator;



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


    public void setWristVoltage(double voltage) {
        io.setWristVolatage(voltage);
    }



    public Command runCoralIntake() {
        return Commands.runEnd(() -> setCoralVoltage(6), () -> setCoralVoltage(0));
    }

    public Command runCoralOuttake() {
        return Commands.runEnd(() -> setCoralVoltage(-6), () -> setCoralVoltage(0));
    }

    public Command wristDown() {
        return Commands.runEnd(() -> setWristVoltage(-5), () -> setWristVoltage(0));
    }

    public Command wristUp() {
        return Commands.runEnd(() -> setWristVoltage(5), () -> setWristVoltage(0));

    }

    // public void runCoralWristManually(CommandXboxController controller) {
    // controller.leftBumper()
    // .whileTrue(() -> setWristAngle(getWristAngle().minus(Rotation2d.fromDegrees(2))));
    // }

}
