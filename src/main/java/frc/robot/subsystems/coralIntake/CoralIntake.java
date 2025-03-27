package frc.robot.subsystems.CoralIntake;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Coral Intake Subsystem
 */
public class CoralIntake extends SubsystemBase {
    private CoralIntakeIO io;
    private CoralIntakeInputsAutoLogged inputs = new CoralIntakeInputsAutoLogged();


    /**
     * Coral Scoring subsystem
     *
     * @param io IO for the Swerve
     */
    public CoralIntake(CoralIntakeIO io) {
        this.io = io;
        io.updateInputs(inputs);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Coral", inputs);
    }

    /**
     * Set motor power for the coral intake/outake
     *
     * @param voltage voltage for the motor for intake/outake
     */
    public void setCoralVoltage(double voltage) {
        Logger.recordOutput("Coral/Voltage", voltage);
        io.setCoralVoltage(voltage);
    }


    /**
     * set motor power for the wrist
     *
     * @param voltage voltage for the motor for the wrist
     */
    public void setWristVoltage(double voltage) {
        io.setWristVolatage(voltage);
    }



    /**
     * Command for the intake set intake voltage
     *
     * @return Command
     */
    public Command runCoralIntake() {
        return Commands.runEnd(() -> setCoralVoltage(3), () -> setCoralVoltage(0));
    }

    /**
     * Command for the outake set outake voltage
     *
     * @return Command
     */
    public Command runCoralOuttake() {
        return Commands.runEnd(() -> setCoralVoltage(-3), () -> setCoralVoltage(0));
    }

    /**
     * Command for the wrist going down set the wrist volage
     *
     * @return Command
     */
    public Command wristDown() {
        return Commands.runEnd(() -> setWristVoltage(-10), () -> setWristVoltage(0));
    }

    /**
     * Command for the wrist going up set the wrist voltage
     *
     * @return Command
     */
    public Command wristUp() {
        return Commands.runEnd(() -> setWristVoltage(10), () -> setWristVoltage(0));

    }

    // public void runCoralWristManually(CommandXboxController controller) {
    // controller.leftBumper()
    // .whileTrue(() -> setWristAngle(getWristAngle().minus(Rotation2d.fromDegrees(2))));
    // }

}
