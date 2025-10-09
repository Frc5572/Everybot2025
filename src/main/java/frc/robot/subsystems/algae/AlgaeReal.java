package frc.robot.subsystems.algae;

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

/**
 * Alage real class
 */
public class AlgaeReal implements AlgaeIO {

    public SparkMax algaeintakeMotor =
        new SparkMax(Constants.AlgaeSubsystem.kalgaeIntakeMotorCanId, MotorType.kBrushless);
    public SparkMax algaeWristMotor =
        new SparkMax(Constants.AlgaeSubsystem.kalgaeWristMotorCanId, MotorType.kBrushless);
    public SparkClosedLoopController wristController = algaeWristMotor.getClosedLoopController();
    public RelativeEncoder encoder = algaeWristMotor.getEncoder();

    public SparkMaxConfig algaeintakeconfig = new SparkMaxConfig();
    public SparkMaxConfig algaeWristMotorconfig = new SparkMaxConfig();



    /**
     * Algae Real
     */
    public AlgaeReal() {
        encoder.setPosition(0.0);

        algaeWristMotorconfig.idleMode(IdleMode.kBrake);
        algaeWristMotorconfig.closedLoop.positionWrappingEnabled(false);
        algaeWristMotor.configure(algaeWristMotorconfig, ResetMode.kNoResetSafeParameters,
            PersistMode.kPersistParameters);

        algaeintakeconfig.idleMode(IdleMode.kBrake);
        algaeintakeMotor.configure(algaeintakeconfig, ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(AlgaeInputs inputs) {
        inputs.algaewristPosition = encoder.getPosition();
    }

    @Override
    public void setAlgaeVoltage(double voltage) {
        algaeintakeMotor.setVoltage(voltage);
    }

    @Override
    public void setAlgaeWristVoltage(double voltage) {
        algaeWristMotor.setVoltage(voltage);
    }

    @Override
    public void setPosition(double setPoint, double feedforward) {
        wristController.setReference(setPoint, ControlType.kPosition, ClosedLoopSlot.kSlot0,
            feedforward);
    }

    @Override
    public void setPID(double kP, double kI, double kD) {
        algaeWristMotorconfig.closedLoop.pidf(kP, kI, kD, 0.0, ClosedLoopSlot.kSlot0);
        algaeWristMotor.configure(algaeWristMotorconfig, ResetMode.kNoResetSafeParameters,
            PersistMode.kNoPersistParameters);
    }

}
