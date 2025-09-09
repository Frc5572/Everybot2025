package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import frc.robot.Constants;

public class ModuleSparkMax implements ModuleIO {

    private final SparkMax driveMotor;
    private final SparkMax angleMotor;
    private final CANcoder angleAbsoluteEncoder;
    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder angleEncoder;

    private double kS = 0;
    private double kV = 0;

    private CANcoderConfiguration absoluteEncoderConfig = new CANcoderConfiguration();

    private StatusSignal<Angle> absoluteEncoderValue;

    private SparkMaxConfig angleConfig = new SparkMaxConfig();
    private SparkMaxConfig driveConfig = new SparkMaxConfig();

    private SparkClosedLoopController angleController;
    private SparkClosedLoopController driveController;

    public ModuleSparkMax(int idx) {
        ModuleConstants constants = Constants.Swerve.moduleConstants[idx];
        driveMotor = new SparkMax(constants.driveMotorId(), MotorType.kBrushless);
        angleMotor = new SparkMax(constants.driveMotorId(), MotorType.kBrushless);
        angleAbsoluteEncoder = new CANcoder(constants.encoderId());
        driveEncoder = driveMotor.getEncoder();
        angleEncoder = angleMotor.getEncoder();

        configAbsoluteEncoder();
        configAngleMotor();
        configDriveMotor();
    }

    private void configAngleMotor() {
        /* Motor Inverts and Neutral Mode */
        angleConfig.inverted(Constants.Swerve.angleMotorInvert).idleMode(IdleMode.kBrake)
            .voltageCompensation(12);
        // /* PID Config */
        angleConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(Constants.Swerve.angleKP, Constants.Swerve.angleKI, Constants.Swerve.angleKD)
            .positionWrappingEnabled(true).positionWrappingMinInput(-0.5)
            .positionWrappingMaxInput(0.5)
            .outputRange(Constants.Swerve.angleMinOutput, Constants.Swerve.angleMaxOutput);
        angleConfig.encoder.positionConversionFactor(Constants.Swerve.angleGearRatio)
            .velocityConversionFactor(Constants.Swerve.angleGearRatio);

        this.angleController = angleMotor.getClosedLoopController();

        this.angleMotor.configure(angleConfig, ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);
    }

    private void configDriveMotor() {
        driveConfig.inverted(Constants.Swerve.driveMotorInvert).idleMode(IdleMode.kBrake);
        driveConfig.encoder.positionConversionFactor(1.0 / Constants.Swerve.driveGearRatio)
            .velocityConversionFactor(1.0 / Constants.Swerve.driveGearRatio);
        driveConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(
            Constants.Swerve.driveKP, Constants.Swerve.driveKI, Constants.Swerve.driveKD,
            ClosedLoopSlot.kSlot0);
        this.driveController = driveMotor.getClosedLoopController();
        this.driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);
    }

    private void configAbsoluteEncoder() {
        absoluteEncoderConfig.MagnetSensor.SensorDirection = Constants.Swerve.cancoderInvert;
        absoluteEncoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
        angleAbsoluteEncoder.getConfigurator().apply(absoluteEncoderConfig);

        absoluteEncoderValue = angleAbsoluteEncoder.getAbsolutePosition();
    }

    @Override
    public void updateInputs(ModuleInputs inputs) {
        BaseStatusSignal.refreshAll(absoluteEncoderValue);

        // Absolute Encoder
        inputs.absoluteEncoderConnected = angleAbsoluteEncoder.isConnected();
        inputs.angleAbsolutePositionRad = absoluteEncoderValue.getValue().in(Radians);

        // Drive Motor
        inputs.driveConnected = !isMotorDisconnected(driveMotor);
        inputs.driveAppliedVolts = driveMotor.getBusVoltage();
        inputs.drivePositionRad = Units.rotationsToRadians(driveEncoder.getPosition());
        inputs.driveVelocityRadPerSec = RPM.of(driveEncoder.getVelocity()).in(RadiansPerSecond);
        // not sure if there's a way in REVLib to distinguish supply vs torque current. Docs
        // indicate this returns torque current.
        inputs.driveSupplyCurrentAmps = driveMotor.getOutputCurrent();
        inputs.driveTorqueCurrentAmps = driveMotor.getOutputCurrent();

        // Angle Motor
        inputs.angleConnected = !isMotorDisconnected(angleMotor);
        inputs.angleAppliedVolts = angleMotor.getBusVoltage();
        inputs.anglePositionRad = Units.rotationsToRadians(angleEncoder.getPosition());
        inputs.angleVelocityRadPerSec = Units.rotationsToRadians(angleEncoder.getVelocity());
        inputs.angleSupplyCurrentAmps = angleMotor.getOutputCurrent();
        inputs.angleTorqueCurrentAmps = angleMotor.getOutputCurrent();
    }

    private boolean isMotorDisconnected(SparkMax max) {
        // TODO check via lastError()
        return false;
    }

    @Override
    public void setAnglePosition(Rotation2d angle) {
        angleEncoder.setPosition(angle.getRotations());
    }

    @Override
    public void runDriveVelocity(double velocityRadPerSec, double feedforward) {
        feedforward += kV * velocityRadPerSec;
        if (velocityRadPerSec > 1e-5) {
            feedforward += kS;
        } else if (velocityRadPerSec < -1e-5) {
            feedforward -= kS;
        }
        driveController.setReference(velocityRadPerSec, SparkBase.ControlType.kVelocity,
            ClosedLoopSlot.kSlot0, feedforward, ArbFFUnits.kVoltage);
    }

    @Override
    public void runAnglePosition(Rotation2d rotation) {
        angleController.setReference(rotation.getRadians(), SparkBase.ControlType.kPosition);
    }

    @Override
    public void setDrivePID(double kP, double kI, double kD, double kS, double kV) {
        driveConfig.closedLoop.pid(kP, kI, kD, ClosedLoopSlot.kSlot0);
        this.driveMotor.configure(driveConfig, ResetMode.kNoResetSafeParameters,
            PersistMode.kNoPersistParameters);
        this.kS = kS;
        this.kV = kV;
    }

    @Override
    public void setAnglePID(double kP, double kI, double kD) {
        angleConfig.closedLoop.pid(kP, kI, kD);
        this.angleMotor.configure(angleConfig, ResetMode.kNoResetSafeParameters,
            PersistMode.kNoPersistParameters);
    }

    @Override
    public void setBrakeMode(boolean enabled) {
        driveConfig.idleMode(enabled ? IdleMode.kBrake : IdleMode.kCoast);
        this.driveMotor.configure(driveConfig, ResetMode.kNoResetSafeParameters,
            PersistMode.kNoPersistParameters);
    }

    @Override
    public void runDriveCharacterization(double volts) {
        driveMotor.setVoltage(volts);
    }

}
