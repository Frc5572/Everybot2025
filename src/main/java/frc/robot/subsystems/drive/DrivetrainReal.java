package frc.robot.subsystems.drive;

/**
 * DrivetrainReal
 */
public class DrivetrainReal implements DrivetrainIO {


    /**
     * } } Drivetrain Real
     */
    // private AHRS gyro = new AHRS(Constants.Swerve.navXID);

    public DrivetrainReal() {}

    @Override
    public void updateInputs(DrivetrainIOInputs inputs) {
        // inputs.yaw = gyro.getYaw();
        // inptus.pitch = gyro.getPitch();


    }

    /**
     * Drive Voltage
     */
    public void setDriveVoltage(double lvolts, double rvolts) {}

}
