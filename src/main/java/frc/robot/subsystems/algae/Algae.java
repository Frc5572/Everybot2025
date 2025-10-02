package frc.robot.subsystems.algae;

import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Algae Subsystem
 */
public class Algae extends SubsystemBase {
    private AlgaeIO io;
    private AlgaeInputsAutoLogged inputs = new AlgaeInputsAutoLogged();

    private double kP = 3.0;
    private double kI = 0.0;
    private double kD = 0.0;
    private double kFF = 0.0;
    private double kS = 0.0;

    private static final String kPString = "Algae/kP";
    private static final String kIString = "Algae/kI";
    private static final String kDString = "Algae/kD";
    private static final String kFString = "Algae/kFF";
    private static final String kSString = "Algae/kS";

    /**
     * Algae subsystem
     *
     * @param io IO for the Algae
     */
    public Algae(AlgaeIO io) {
        this.io = io;
        io.updateInputs(inputs);

        SmartDashboard.putNumber(kPString, kP);
        SmartDashboard.putNumber(kIString, kI);
        SmartDashboard.putNumber(kDString, kD);
        SmartDashboard.putNumber(kFString, kFF);
        SmartDashboard.putNumber(kSString, kS);
    }

    private boolean positionControl;
    private double controlParam;

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Algae", inputs);

        double newKp = SmartDashboard.getNumber(kPString, 0.0);
        double newKd = SmartDashboard.getNumber(kIString, 0.0);
        double newKi = SmartDashboard.getNumber(kDString, 0.0);
        double newKff = SmartDashboard.getNumber(kFString, 0.0);
        double newKs = SmartDashboard.getNumber(kSString, 0.0);

        if (newKp != kP || newKd != kD || newKi != kI || newKff != kFF || newKs != kS) {
            kP = newKp;
            kI = newKi;
            kD = newKd;
            kFF = newKff;
            kS = newKs;

            io.setPID(newKp, newKi, newKd);
        }

        if (positionControl) {
            double ff = Math.signum(inputs.algaewristPosition - controlParam) * kS;
            io.setPosition(controlParam, ff);
        } else {
            io.setAlgaeWristVoltage(controlParam);
        }
    }

    private void setPosition(double position) {
        this.positionControl = true;
        this.controlParam = position;
    }

    private void setVoltage(double voltage) {
        this.positionControl = false;
        this.controlParam = voltage;
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
     * Command for the intake set intake voltage
     *
     * @return Command
     */
    public Command runAlgaeIntake() {
        return Commands.runEnd(() -> setAlgaeVoltage(Constants.AlgaeSubsystem.INTAKE_VOLTAGE),
            () -> setAlgaeVoltage(0));
    }

    /**
     * Command for the outake set outake voltage
     *
     * @return Command
     */
    public Command runAlgaeOuttake() {
        return Commands.runEnd(() -> setAlgaeVoltage(Constants.AlgaeSubsystem.OUTTAKE_VOLTAGE),
            () -> setAlgaeVoltage(0));
    }

    public Command setVoltage(DoubleSupplier supplier) {
        return this.run(() -> {
            this.setVoltage(supplier.getAsDouble());
        });
    }

    public Command moveTo(DoubleSupplier angle) {
        return this.run(() -> {
            this.setPosition(angle.getAsDouble());
        });
    }
}
