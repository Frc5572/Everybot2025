package frc.robot.subsystems.SwerveModule;

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

    /*
     * private final SimpleMotorFeedforward driveFeedForward = new SimpleMotorFeedforward(
     * Constants.Swerve.driveKS, Constants.Swerve.driveKV, Constants.Swerve.driveKA);
     * 
     * drive motor control requests
     * 
     * 
     * private final DutyCycleOut driveDutyCycle = new DutyCycleOut(0); private final
     * VelocityVoltage driveVelocity = new VelocityVoltage(0);
     */

    public SwerveModule(int moduleNum, int driveMotorID, int angleMotorID, int cancoderID, Rotation2d angleOffset, SwerveModuleIO io) {
        Rotation2d angleOffset, SwerveModuleIO io) {
        this.io = io;

        this.moduleNum= moduleNum;

        this.angleOffset = angleOffset;

        io.updateInputs(inputs);
        try {
            Thread.sleep(2000);
        }
        catch(InterruptedException e){
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
        if (isOpenLoop) {
            driveDutyCycle.Output = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;
            io.setDriveMotor(driveDutyCycle);
        } else {
            driveVelocity.Velocity = Conversions.MetersPerSecondToRotationPerSecond(
                desiredState.speedMetersPerSecond, Constants.Swerve.wheelCircumference);
            driveVelocity.FeedForward =
                driveFeedForward.calculate(desiredState.speedMetersPerSecond);
            io.setDriveMotor(driveVelocity);
        }
    }

    public Rotation2d getCANcoder() {
        returnRotation2d.fromRotations(inputs.absolutePositionAngleEncoder.in(Rotations));
    }

    public void resetToAbsolute() {
        double absolutePosition = getCANcoder().getRotations() - angleOffset.getRotations();
        io.setPositionAngleMotor(absolutePosition);
        inputs.angleMotorSelectedPosition = Rotations.of(absolutePosition);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
            Conversions.rotationperSecondToMetersPerSecond(
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
