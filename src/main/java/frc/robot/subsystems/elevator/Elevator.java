package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Meters;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Elevator subsystem */
public class Elevator extends SubsystemBase {

    private final ElevatorIO io;
    private final ElevatorInputsAutoLogged inputs = new ElevatorInputsAutoLogged();

    private double kP = 20.0;
    private double kI = 0.0;
    private double kD = 0.0;
    private double kFF = 0.0;
    private double kS = 0.0;

    private static final String kPString = "Elevator/kP";
    private static final String kIString = "Elevator/kI";
    private static final String kDString = "Elevator/kD";
    private static final String kFString = "Elevator/kFF";
    private static final String kSString = "Elevator/kS";

    /** Elevator initlizer */
    public Elevator(ElevatorIO io) {
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
        Logger.processInputs("Elevator", inputs);

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

            io.setPID(newKp, newKi, newKd, newKff);
        }

        if (positionControl) {
            double ff = Math.signum(inputs.position.in(Meters) - controlParam) * kS;
            io.setPosition(controlParam, ff);
        } else {
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

    public Command moveTo(DoubleSupplier height) {
        return this.run(() -> {
            this.setPosition(height.getAsDouble());
        });
    }

    public Command setVoltage(DoubleSupplier v) {
        return runEnd(() -> this.setVoltage(v.getAsDouble()), (() -> this.setVoltage(0.0)));
    }
}
