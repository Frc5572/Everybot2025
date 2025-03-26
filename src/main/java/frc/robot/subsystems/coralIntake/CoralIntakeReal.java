package frc.robot.subsystems.CoralIntake;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
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


    public SparkMaxConfig intakeconfig = new SparkMaxConfig();
    public SparkMaxConfig coralWristMotorconfig = new SparkMaxConfig();



    public CoralIntakeReal() {
        coralWristMotorconfig.idleMode(IdleMode.kBrake);
        coralWristMotor.configure(coralWristMotorconfig, ResetMode.kNoResetSafeParameters,
            PersistMode.kPersistParameters);

        intakeconfig.idleMode(IdleMode.kBrake);
        intakeMotor.configure(intakeconfig, ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(CoralIntakeInputs inputs) {

    }

    @Override
    public void setCoralVoltage(double voltage) {
        intakeMotor.setVoltage(voltage);
    }

    @Override
    public void setWristVolatage(double voltage) {
        coralWristMotor.setVoltage(voltage);
    }



}
