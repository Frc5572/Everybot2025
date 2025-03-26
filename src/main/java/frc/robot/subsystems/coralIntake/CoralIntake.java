package frc.robot.subsystems.CoralIntake;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.coralIntake.CoralIntakeInputsAutoLogged;

/** Coral Intake Subsystem */
public class CoralIntake extends SubsystemBase {
    private CoralIntakeIO io;
    private CoralIntakeInputsAutoLogged inputs = new CoralIntakeInputsAutoLogged();


    /** Coral Intake initlizer */
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
        return Commands.runEnd(() -> setCoralVoltage(12), () -> setCoralVoltage(0));
    }

    public Command runCoralOuttake() {
        return Commands.runEnd(() -> setCoralVoltage(-12), () -> setCoralVoltage(0));
    }

    public Command wristDown() {
        return Commands.runEnd(() -> setWristVoltage(-12), () -> setWristVoltage(0));
    }

    public Command wristUp() {
        return Commands.runEnd(() -> setWristVoltage(12), () -> setWristVoltage(0));

    }

    // public void runCoralWristManually(CommandXboxController controller) {
    // controller.leftBumper()
    // .whileTrue(() -> setWristAngle(getWristAngle().minus(Rotation2d.fromDegrees(2))));
    // }

}
