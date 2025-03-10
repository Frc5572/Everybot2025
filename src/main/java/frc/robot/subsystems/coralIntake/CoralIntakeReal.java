package frc.robot.subsystems.coralIntake;

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



    private SparkMaxConfig intakeconfig = new SparkMaxConfig();


    public CoralIntakeReal() {
        intakeconfig.idleMode(IdleMode.kBrake);
        intakeMotor.configure(intakeconfig, ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);
    }

    // what do I update?

    public void updateInputs(CoralIntakeInputs inputs) {


    }

    public void setCoralVoltage(double voltage) {
        intakeMotor.setVoltage(voltage);
    }



}
