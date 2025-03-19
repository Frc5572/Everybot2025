package frc.lib.util.swerve;

import org.littletonrobotics.junction.AutoLog;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

/**
 * Swerve Module IO
 */
public interface SwerveModuleIO {

    /**
     * Swerve Module Inputs Class
     */
    @AutoLog
    public static class SwerveModuleInputs {
        public Angle driveMotorSelectedPosition;
        public AngularVelocity driveMotorSelectedSensorVelocity;
        public Angle angleMotorSelectedPosition;
        public Angle absolutePositionAngleEncoder;


    }

    public default void updateInputs(SwerveModuleInputs inputs) {}

    public default void setDriveMotor(double v) {}

    public default void setAngleMotor(double v) {}

    public default void setAngleSelectedSensorPosition(double angle) {}

    public default void setPositionAngleMotor(double absolutePosition) {}


}
