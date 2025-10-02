package frc.robot.subsystems.coral;

import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;


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

    private Command moveTo(DoubleSupplier angle) {
        final double[] angleStore = new double[] {0.0};
        return runOnce(() -> {
            angleStore[0] = angle.getAsDouble();
        }).andThen(runOnce(() -> {
            io.setWristSetPoint(angleStore[0]);
        })).andThen(Commands.waitUntil(() -> Math.abs(inputs.wristAngle - angleStore[0]) < 1));
    }

    public Command controlWristCommand(CommandXboxController controller) {
        return moveTo(() -> (io.getWristRotations() * 360) % 360 + controller.getLeftY());
    }

    public Command wristVoltage(DoubleSupplier voltage) {
        return this.run(() -> {
            io.setWristVolatage(voltage.getAsDouble());
        });
    }

    /**
     * Set motor power for the coral intake/outake
     *
     * @param voltage voltage for the motor for intake/outake
     */
    private void setCoralVoltage(double voltage) {
        Logger.recordOutput("Coral/Voltage", voltage);
        io.setCoralVoltage(voltage);
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

}
