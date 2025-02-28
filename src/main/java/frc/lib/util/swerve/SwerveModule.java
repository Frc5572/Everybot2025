package frc.lib.util.swerve;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.math.Conversions;
import frc.robot.Constants;

public class SwerveModule {
    public int moduleNum;
    private Rotation2d angleOffset;

    private SwerveModuleIO io;
    private SwerveModuleInputsAutoLogged inputs = new SwerveModuleInputsAutoLogged();



    public SwerveModule(int moduleNum, int driveMotorID, int angleMotorID, int cancoderID,
        Rotation2d angleOffset, SwerveModuleIO io) {
        this.io = io;

        this.moduleNum = moduleNum;

        this.angleOffset = angleOffset;

        io.updateInputs(inputs);
        try {
            Thread.sleep(2000);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();;
        }
        resetToAbsolute();
        Logger.processInputs("SwerveModule" + moduleNum, inputs);
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("SwerveModule" + moduleNum, inputs);
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        desiredState.optimize(getState().angle);
        io.setAngleMotor(desiredState.angle.getRotations());
        setSpeed(desiredState, isOpenLoop);
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {

        io.setDriveMotor(desiredState.speedMetersPerSecond * Constants.Swerve.wheelCircumference);
    }

    public Rotation2d getCANcoder() {
        return Rotation2d.fromRotations(inputs.absolutePositionAngleEncoder.in(Rotations));
    }

    public void resetToAbsolute() {
        double absolutePosition = getCANcoder().getRotations() - angleOffset.getRotations();
        io.setPositionAngleMotor(absolutePosition);
        inputs.angleMotorSelectedPosition = Rotations.of(absolutePosition);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
            Conversions.rotationPerSecondToMetersPerSecond(
                inputs.driveMotorSelectedSensorVelocity.in(RotationsPerSecond),
                Constants.Swerve.wheelCircumference),
            Rotation2d.fromRotations(inputs.angleMotorSelectedPosition.in(Rotations)));
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(


            Conversions.rotationsToMeters(inputs.driveMotorSelectedPosition.in(Rotations),
                Constants.Swerve.wheelCircumference),
            Rotation2d.fromRotations(inputs.angleMotorSelectedPosition.in(Rotations)));
    }
}
