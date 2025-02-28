package frc.robot.subsystems.swerve;

import frc.robot.Constants;

public class SwerveReal implements SwerveIO {
    private AHRS gyro = new AHRS(Constants.Swerve.navXID);

    public SwerveReal() {}

    @Override
    public void updateInputs(SwerveInputs inputs){
        inputs.yaw = gyro.getYaw();
    }
}
