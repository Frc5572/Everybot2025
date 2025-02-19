package frc.robot.subsystems.SwerveModule;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

public class SwerveModuleReal implements SwerveModuleIO {

    private SparkMax mDriveMotor;
    private SparkMax mAngleMotor;
    // private SparkClosedLoopController angleController; look this up
    private CANcoder angleEncoder;
    public RelativeEncoder angleMotorEncoder;
    private CANcoderConfiguration swerveCANcoderConfig = new CANcoderConfiguration();

    private StatusSignal<Angle> driveMotorSelectedPosition;
    private StatusSignal<AngularVelocity> driveMotorSelectedSensorVelocity;
    private StatusSignal<Angle> absolutePositionAngleEncoder;

    private SparkMaxConfig config = new SparkMaxConfig();
    private int moduleNum;



}
