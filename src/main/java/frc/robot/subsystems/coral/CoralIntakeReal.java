package frc.robot.subsystems.coral;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants;

/** Coral Intake Real Class */
public class CoralIntakeReal implements CoralIntakeIO {

    public SparkMax intakeMotor =
        new SparkMax(Constants.CoralSubsystem.kIntakeMotorCanId, MotorType.kBrushless);
    public SparkMax coralWristMotor =
        new SparkMax(Constants.CoralSubsystem.kcoralWristMotorCanId, MotorType.kBrushless);
    public SparkClosedLoopController wristController = coralWristMotor.getClosedLoopController();
    public RelativeEncoder encoder = coralWristMotor.getEncoder();
    public SparkMaxConfig intakeconfig = new SparkMaxConfig();
    public SparkMaxConfig coralWristMotorconfig = new SparkMaxConfig();

    /**
     * Coral Intake Real
     */
    public CoralIntakeReal() {
        encoder.setPosition(0.0);
        coralWristMotorconfig.idleMode(IdleMode.kBrake);
        coralWristMotorconfig.closedLoop.positionWrappingEnabled(false);
        coralWristMotorconfig.closedLoop.pidf(Constants.CoralSubsystem.KP,
            Constants.CoralSubsystem.KI, Constants.CoralSubsystem.KD, Constants.CoralSubsystem.FF);
        coralWristMotorconfig.inverted(true);
        coralWristMotor.configure(coralWristMotorconfig, ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);
        intakeconfig.idleMode(IdleMode.kBrake);
        intakeMotor.configure(intakeconfig, ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);
    }



    @Override
    public void updateInputs(CoralIntakeInputs inputs) {
        inputs.wristAngle = encoder.getPosition();
    }

    @Override
    public void setCoralVoltage(double voltage) {
        intakeMotor.setVoltage(voltage);
    }

    @Override
    public void setVoltage(double voltage) {
        coralWristMotor.setVoltage(voltage);
    }

    @Override
    public void setPosition(double setPoint, double ff) {
        wristController.setReference(setPoint, ControlType.kPosition, ClosedLoopSlot.kSlot0, ff);
    }



    @Override
    public void setPID(double kP, double kI, double kD) {
        coralWristMotorconfig.closedLoop.pidf(kP, kI, kD, 0.0, ClosedLoopSlot.kSlot0);
        coralWristMotor.configure(coralWristMotorconfig, ResetMode.kNoResetSafeParameters,
            PersistMode.kNoPersistParameters);
    }
}
