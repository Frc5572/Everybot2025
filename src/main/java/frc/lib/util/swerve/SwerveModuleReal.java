package frc.lib.util.swerve;

import static edu.wpi.first.units.Units.Rotations;
import org.littletonrobotics.junction.Logger;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import frc.robot.Constants;

public class SwerveModuleReal implements SwerveModuleIO {

    private SparkMax mDriveMotor;
    private SparkMax mAngleMotor;
    // private SparkClosedLoopController angleController; look this up
    private CANcoder angleEncoder;
    public RelativeEncoder angleMotorEncoder;
    private CANcoderConfiguration swerveCANcoderConfig = new CANcoderConfiguration();

    private StatusSignal<Angle> absolutePositionAngleEncoder;

    private SparkMaxConfig angleconfig = new SparkMaxConfig();
    private SparkMaxConfig driveconfig = new SparkMaxConfig();
    private int moduleNum;
    private SparkClosedLoopController angleController;
    private SparkClosedLoopController driveController;



    public SwerveModuleReal(int moduleNum, int driveMotorID, int angleMotorID, int cancoderID,
        Rotation2d angleOffset) {
        this.moduleNum = moduleNum;

        mDriveMotor = new SparkMax(driveMotorID, MotorType.kBrushless);
        mAngleMotor = new SparkMax(angleMotorID, MotorType.kBrushless);
        angleEncoder = new CANcoder(cancoderID);
        angleMotorEncoder = mAngleMotor.getEncoder();

        configAngleEncoder();
        configAngleMotor();
        configDriveMotor();

        absolutePositionAngleEncoder = angleEncoder.getAbsolutePosition();
    }

    private void configAngleMotor() {
        /* Angle Motor Config */

        /* Motor Inverts and Neutral Mode */
        angleconfig.inverted(false).idleMode(IdleMode.kBrake).voltageCompensation(12);
        // /* PID Config */
        angleconfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(Constants.Swerve.angleKP, Constants.Swerve.angleKI, Constants.Swerve.angleKD)
            .positionWrappingEnabled(true).positionWrappingMinInput(-0.5)
            .positionWrappingMaxInput(0.5)
            .outputRange(Constants.Swerve.angleMinOutput, Constants.Swerve.angleMaxOutput);
        angleconfig.encoder.positionConversionFactor(Constants.Swerve.angleGearRatio)
            .velocityConversionFactor(Constants.Swerve.angleGearRatio);

        this.angleController = mAngleMotor.getClosedLoopController();

        this.mAngleMotor.configure(angleconfig, ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);
    }

    private void configDriveMotor() {
        driveconfig.inverted(true).idleMode(IdleMode.kBrake);
        driveconfig.encoder.positionConversionFactor(Constants.Swerve.driveGearRatio)
            .velocityConversionFactor(Constants.Swerve.driveGearRatio);
        driveconfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(Constants.Swerve.driveKP, Constants.Swerve.driveKI, Constants.Swerve.driveKD);
        this.driveController = mDriveMotor.getClosedLoopController();
        this.mDriveMotor.configure(driveconfig, ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);
    }

    private void configAngleEncoder() {
        /* Angle Encoder Config */
        swerveCANcoderConfig.MagnetSensor.SensorDirection = Constants.Swerve.cancoderInvert;
        swerveCANcoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
        angleEncoder.getConfigurator().apply(swerveCANcoderConfig);
    }



    @Override
    public void setAngleMotor(double v) {
        angleController.setReference(v, SparkBase.ControlType.kPosition);
    }


    @Override
    public void setDriveMotor(double v) {
        driveController.setReference(v, SparkBase.ControlType.kVelocity);
    }

    @Override
    public void updateInputs(SwerveModuleInputs inputs) {
        BaseStatusSignal.refreshAll(absolutePositionAngleEncoder);
        inputs.angleMotorSelectedPosition = Rotations.of(angleMotorEncoder.getPosition());
        inputs.absolutePositionAngleEncoder = absolutePositionAngleEncoder.getValue();
    }


    @Override
    public void setPositionAngleMotor(double absolutePosition) {
        REVLibError a = angleMotorEncoder.setPosition(absolutePosition);
        Logger.recordOutput("SetPositionAngleError/" + moduleNum, a);
    }



}
