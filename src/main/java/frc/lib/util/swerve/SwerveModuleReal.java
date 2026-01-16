package frc.lib.util.swerve;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import org.littletonrobotics.junction.Logger;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import frc.robot.Constants;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;

/**
 * Swerve Module Real
 */
public class SwerveModuleReal implements SwerveModuleIO {

    private SparkMax mDriveMotor;
    private SparkMax mAngleMotor;
    public RelativeEncoder angleMotorEncoder;
    public RelativeEncoder driveMotorEncoder;

    private StatusSignal<Angle> absolutePositionAngleEncoder;

    private SparkMaxConfig angleconfig = new SparkMaxConfig();
    private SparkMaxConfig driveconfig = new SparkMaxConfig();
    private int moduleNum;
    private SparkClosedLoopController angleController;
    private SparkClosedLoopController driveController;

    /**
     * Swerve Module Real Constructor
     *
     * @param moduleNum Module number
     * @param driveMotorID ID of the Drive motor
     * @param angleMotorID ID of the Angle motor
     * @param cancoderID ID of the CANcoder
     * @param angleOffset The offset angle of the module
     */
    public SwerveModuleReal(int moduleNum, int driveMotorID, int angleMotorID, int cancoderID,
        Rotation2d angleOffset) {
        this.moduleNum = moduleNum;

        mDriveMotor = new SparkMax(driveMotorID, MotorType.kBrushless);
        mAngleMotor = new SparkMax(angleMotorID, MotorType.kBrushless);
        // angleEncoder = new CANcoder(cancoderID);
        angleMotorEncoder = mAngleMotor.getEncoder();
        driveMotorEncoder = mDriveMotor.getEncoder();

        // configAngleEncoder();
        configAngleMotor();
        configDriveMotor();

        // absolutePositionAngleEncoder = angleEncoder.getAbsolutePosition();
    }

    private void configAngleMotor() {
        /* Angle Motor Config */

        /* Motor Inverts and Neutral Mode */
        angleconfig.inverted(Constants.Swerve.angleMotorInvert).idleMode(IdleMode.kBrake)
            .voltageCompensation(12);
        // /* PID Config */
        angleconfig.closedLoop.feedbackSensor(com.revrobotics.spark.FeedbackSensor.kPrimaryEncoder)
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
        driveconfig.inverted(Constants.Swerve.driveMotorInvert).idleMode(IdleMode.kBrake);
        driveconfig.encoder.positionConversionFactor(1.0 / Constants.Swerve.driveGearRatio)
            .velocityConversionFactor(1.0 / Constants.Swerve.driveGearRatio);
        driveconfig.closedLoop.feedbackSensor(com.revrobotics.spark.FeedbackSensor.kPrimaryEncoder)
            .pid(Constants.Swerve.driveKP, Constants.Swerve.driveKI, Constants.Swerve.driveKD);
        this.driveController = mDriveMotor.getClosedLoopController();
        driveconfig.smartCurrentLimit(40, 40);
        this.mDriveMotor.configure(driveconfig, ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);
    }

    @Override
    public void setAngleMotor(double v) {
        angleController.setSetpoint(v, SparkBase.ControlType.kPosition);
    }


    @Override
    public void setDriveMotor(double v) {
        mDriveMotor.setVoltage(v);
    }

    @Override
    public void updateInputs(SwerveModuleInputs inputs) {
        BaseStatusSignal.refreshAll(absolutePositionAngleEncoder);
        inputs.driveMotorSelectedPosition = Rotations.of(driveMotorEncoder.getPosition());
        inputs.driveMotorSelectedSensorVelocity = RPM.of(driveMotorEncoder.getVelocity());
        inputs.angleMotorSelectedPosition = Rotations.of(angleMotorEncoder.getPosition());
        inputs.absolutePositionAngleEncoder = Rotations.of(0);
    }


    @Override
    public void setPositionAngleMotor(double absolutePosition) {
        REVLibError a = angleMotorEncoder.setPosition(absolutePosition);
        Logger.recordOutput("SetPositionAngleError/" + moduleNum, a);
    }



}
