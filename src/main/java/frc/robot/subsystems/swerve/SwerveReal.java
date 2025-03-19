package frc.robot.subsystems.swerve;

import com.studica.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.util.swerve.SwerveModule;
import frc.lib.util.swerve.SwerveModuleReal;
import frc.robot.Constants;

/**
 * Swerve Real Class
 */
public class SwerveReal implements SwerveIO {
    private AHRS gyro = new AHRS(Constants.Swerve.navXID);

    public SwerveReal() {}

    @Override
    public void updateInputs(SwerveInputs inputs) {
        inputs.yaw = gyro.getYaw();
        inputs.pitch = gyro.getPitch();
        inputs.roll = gyro.getRoll();
    }

    @Override
    public SwerveModule createSwerveModule(int moduleNum, int driveMotorID, int angleMotorID,
        int cancoderID, Rotation2d angleOffset) {
        return new SwerveModule(moduleNum, driveMotorID, angleMotorID, cancoderID, angleOffset,
            new SwerveModuleReal(moduleNum, driveMotorID, angleMotorID, cancoderID, angleOffset));
    }
}
