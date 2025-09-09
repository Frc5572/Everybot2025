package frc.robot.subsystems.swerve;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

public class Module {

    private final ModuleIO io;
    public final ModuleInputsAutoLogged inputs = new ModuleInputsAutoLogged();

    public Module(ModuleIO io) {
        this.io = io;
    }

    public void updateInputs(String key) {
        this.io.updateInputs(inputs);
        Logger.processInputs(key, inputs);
    }

    public void periodic() {

    }

    public void runCharacterization(double volts) {
        io.runDriveCharacterization(volts);
        io.runAnglePosition(Rotation2d.kZero);
    }

    public Rotation2d getAngle() {
        return Rotation2d.fromRadians(inputs.anglePositionRad);
    }

    public double getPositionMeters() {
        return inputs.drivePositionRad * Constants.Swerve.wheelRadius;
    }

    public double getVelocityMetersPerSec() {
        return inputs.driveVelocityRadPerSec * Constants.Swerve.wheelRadius;
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getVelocityMetersPerSec(), getAngle());
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getPositionMeters(), getAngle());
    }

    public double getWheelRadiusCharacterizationPosition() {
        return inputs.drivePositionRad;
    }

    public double getFFCharacterizationVelocity() {
        return Units.radiansToRotations(inputs.driveVelocityRadPerSec);
    }

    public void setBrakeMode(boolean enabled) {
        io.setBrakeMode(enabled);
    }

}
