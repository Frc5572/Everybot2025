package frc.robot.subsystems.Algae;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants;

public class AlgaeReal implements AlgaeIO {

    public SparkMax algaeintakeMotor =
        new SparkMax(Constants.AlgaeSubsystem.kalgaeIntakeMotorCanId, MotorType.kBrushless);
    public SparkMax algaeWristMotor =
        new SparkMax(Constants.AlgaeSubsystem.kalgaeWristMotorCanId, MotorType.kBrushless);


    public SparkMaxConfig algaeintakeconfig = new SparkMaxConfig();
    public SparkMaxConfig algaeWristMotorconfig = new SparkMaxConfig();



    public AlgaeReal() {
        algaeWristMotorconfig.idleMode(IdleMode.kBrake);
        algaeWristMotor.configure(algaeWristMotorconfig, ResetMode.kNoResetSafeParameters,
            PersistMode.kPersistParameters);

        algaeintakeconfig.idleMode(IdleMode.kBrake);
        algaeintakeMotor.configure(algaeintakeconfig, ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(AlgaeInputs inputs) {

    }

    @Override
    public void setAlgaeVoltage(double voltage) {
        algaeintakeMotor.setVoltage(voltage);
    }

    @Override
    public void setAlgaeWristVolatage(double voltage) {
        algaeWristMotor.setVoltage(voltage);
    }



}
