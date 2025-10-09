package frc.robot.subsystems.coral;

import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


/**
 * Coral Intake Subsystem
 */
public class CoralIntake extends SubsystemBase {
    private CoralIntakeIO io;
    private CoralIntakeInputsAutoLogged inputs = new CoralIntakeInputsAutoLogged();

    private double kP = 3.0;
    private double kI = 0.0;
    private double kD = 0.0;
    private double kFF = 0.0;
    private double kS = 0.0;

    private static final String kPString = "Coral/kP";
    private static final String kIString = "Coral/kI";
    private static final String kDString = "Coral/kD";
    private static final String kFString = "Coral/kFF";
    private static final String kSString = "Coral/kS";


    /**
     * Coral Scoring subsystem
     *
     * @param io IO for the Swerve
     */
    public CoralIntake(CoralIntakeIO io) {
        this.io = io;
        io.updateInputs(inputs);

        SmartDashboard.putNumber(kPString, kP);
        SmartDashboard.putNumber(kIString, kI);
        SmartDashboard.putNumber(kDString, kD);
        SmartDashboard.putNumber(kFString, kFF);
        SmartDashboard.putNumber(kSString, kS);

        io.setPID(kP, kI, kD);
    }

    private boolean positionControl;
    private double controlParam;

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Coral", inputs);

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
            double ff = Math.signum(inputs.wristAngle - controlParam) * kS;
            Logger.recordOutput("coralIntake", "position: " + controlParam + ", ff: " + ff);
            io.setPosition(controlParam, ff);
        } else {
            Logger.recordOutput("coralIntake", "voltage: " + controlParam);
            io.setVoltage(controlParam);
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
     * Moves to given measurement
     */
    public Command moveTo(DoubleSupplier angle) {
        return this.run(() -> {
            this.setPosition(angle.getAsDouble());
        });
    }


    /**
     * Moves to given measurement
     */
    public Command wristVoltage(DoubleSupplier voltage) {
        return this.run(() -> {
            this.setVoltage(voltage.getAsDouble());
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
        return Commands.runEnd(() -> setCoralVoltage(-4.5), () -> setCoralVoltage(0));
    }

    /**
     * Command for the outake set outake voltage
     *
     * @return Command
     */
    public Command runCoralOuttake() {
        return Commands.runEnd(() -> setCoralVoltage(4.5), () -> setCoralVoltage(0));
    }

}
