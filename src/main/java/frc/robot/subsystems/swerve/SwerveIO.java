package frc.robot.subsystems.swerve;

import java.util.Optional;
import org.littletonrobotics.junction.AutoLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.util.swerve.SwerveModule;
import frc.lib.util.swerve.SwerveModuleIO;

/**
 * Swerve IO Class
 */
public interface SwerveIO {

    /**
     * Swerve Inputs Class
     */
    @AutoLog
    public static class SwerveInputs {
        public float yaw;
        public float roll;
        public float pitch;

    }

    public default void updateInputs(SwerveInputs inputs) {}

    public default SwerveModule createSwerveModule(int moduleNum, int driveMotorID,
        int angleMotorID, int cancoderID, Rotation2d angleOffset) {
        return new SwerveModule(moduleNum, driveMotorID, angleMotorID, cancoderID, angleOffset,
            new SwerveModuleIO() {});
    }

    public default Optional<Pose2d> getInitialPose() {
        return Optional.empty();
    }

    public default void update(int i, Pose2d pose) {}
}
