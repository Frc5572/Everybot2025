package frc.robot.subsystems.swerve;

import com.studica.frc.AHRS;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

public class GyroNavX implements GyroIO {

    private AHRS gyro = new AHRS(Constants.Swerve.navXID);

    @Override
    public void updateInputs(GyroInputs inputs) {
        inputs.yawRads = Units.degreesToRadians(gyro.getYaw());
    }

}
