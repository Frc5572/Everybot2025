package frc.robot.subsystems.coral;

import org.littletonrobotics.junction.AutoLog;

/**
 * Coral Intake IO
 */
public interface CoralIntakeIO {
    /** Coral Intake inputs */
    @AutoLog
    public static class CoralIntakeInputs {
        public double wristMotorVoltage;
        public double wristMotorAmp;
        public double wristAngle;

    }

    public void updateInputs(CoralIntakeInputs inputs);

    public void setCoralVoltage(double voltage);

    public void setVoltage(double voltage);

    public void setPosition(double setPoint, double feedforward);

    public void setPID(double kP, double kI, double kD);
}
