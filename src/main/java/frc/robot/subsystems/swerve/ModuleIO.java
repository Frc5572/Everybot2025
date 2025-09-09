package frc.robot.subsystems.swerve;

import org.littletonrobotics.junction.AutoLog;
import edu.wpi.first.math.geometry.Rotation2d;

public interface ModuleIO {

    @AutoLog
    public static class ModuleInputs {
        // Drive Motor
        boolean driveConnected = false;
        double drivePositionRad = 0;
        double driveVelocityRadPerSec = 0;
        double driveAppliedVolts = 0;
        double driveSupplyCurrentAmps = 0;
        double driveTorqueCurrentAmps = 0;

        // Angle Motor
        boolean angleConnected = false;
        double anglePositionRad = 0;
        double angleVelocityRadPerSec = 0;
        double angleAppliedVolts = 0;
        double angleSupplyCurrentAmps = 0;
        double angleTorqueCurrentAmps = 0;

        // Absolute Encoder
        boolean absoluteEncoderConnected = false;
        double angleAbsolutePositionRad = 0;
    }

    public void updateInputs(ModuleInputs inputs);

    /**
     * Set angle motor position to given value (in radians). Used to zero angle motors.
     */
    public void setAnglePosition(Rotation2d angle);

    /**
     * Set drive motor setpoint. Target velocity given in radians per second. Feedforward is extra
     * voltage added after applying PID on velocity.
     */
    public void runDriveVelocity(double velocityRadPerSec, double feedforward);

    /**
     * Set angle motor setpoint.
     */
    public void runAnglePosition(Rotation2d rotation);

    /**
     * Set velocity PID values.
     */
    public void setDrivePID(double kP, double kI, double kD, double kS, double kV);

    /**
     * Set angle PID values.
     */
    public void setAnglePID(double kP, double kI, double kD);

    /**
     * Set raw output of drive motor.
     */
    public void runDriveCharacterization(double volts);

    /**
     * Enable/Disable brake mode on drive motor.
     */
    public void setBrakeMode(boolean enabled);

    public static class Empty implements ModuleIO {

        public Empty(int _idx) {}

        @Override
        public void updateInputs(ModuleInputs inputs) {}

        @Override
        public void setAnglePosition(Rotation2d angle) {}

        @Override
        public void runDriveVelocity(double velocityRadPerSec, double feedforward) {}

        @Override
        public void runAnglePosition(Rotation2d rotation) {}

        @Override
        public void setDrivePID(double kP, double kI, double kD, double kS, double kV) {}

        @Override
        public void setAnglePID(double kP, double kI, double kD) {}

        @Override
        public void setBrakeMode(boolean enabled) {}

        @Override
        public void runDriveCharacterization(double volts) {}

    }

}
