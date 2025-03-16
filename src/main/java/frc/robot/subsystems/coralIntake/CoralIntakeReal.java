package frc.robot.subsystems.coralIntake;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants.CoralSubsystemConstants;

public class CoralIntakeReal implements CoralIntakeIO {

    private SparkMax intakeMotor =
        new SparkMax(CoralSubsystemConstants.kIntakeMotorCanId, MotorType.kBrushless);
    private SparkMax coralWristMotor =
        new SparkMax(CoralSubsystemConstants.kcoralWristMotorCanId, MotorType.kBrushless);
    private RelativeEncoder coralWristEncoder = coralWristMotor.getEncoder();



    private SparkMaxConfig intakeconfig = new SparkMaxConfig();
    private SparkMaxConfig coralWristMotorconfig = new SparkMaxConfig();



    public CoralIntakeReal() {
        coralWristMotorconfig.idleMode(IdleMode.kBrake);
        coralWristMotor.configure(coralWristMotorconfig, ResetMode.kNoResetSafeParameters,
            PersistMode.kPersistParameters);
        intakeconfig.idleMode(IdleMode.kBrake);
        intakeMotor.configure(intakeconfig, ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);

        armEncoder.setPosition(0);
    }

    // what do I update?

    public void updateInputs(CoralIntakeInputs inputs) {


    }

    public void setCoralVoltage(double voltage) {
        intakeMotor.setVoltage(voltage);
    }



}
