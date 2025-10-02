package frc.robot.subsystems.algae;

import org.littletonrobotics.junction.AutoLog;

/**
 * Algae IO
 */
public interface AlgaeIO {
    /**
     * Algae inputs
     */
    @AutoLog
    public static class AlgaeInputs {
        public double algaewristMotorVoltage;
        public double algaewristMotorAmp;
        public double algaewristPosition;
    }

    public void updateInputs(AlgaeInputs inputs);

    public void setAlgaeVoltage(double voltage);

    public void setAlgaeWristVoltage(double voltage);

    public void setPosition(double setPoint, double feedforward);

    public void setPID(double kP, double kI, double kD);

}
