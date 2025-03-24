package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Meters;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

public class ElevatorReal implements ElevatorIO {
    private final SparkMax elevatorMotor = new SparkMax(0, MotorType.kBrushless);
    private final SparkBaseConfig elevatorConf = new SparkMaxConfig();
    private final SparkClosedLoopController controller = elevatorMotor.getClosedLoopController();
    private final AbsoluteEncoder encoder = elevatorMotor.getAbsoluteEncoder();

    ElevatorReal() {
        config();
    }



    private void config() {
        elevatorConf.inverted(false);
        elevatorConf.encoder.positionConversionFactor(0);
        elevatorConf.closedLoop.pidf(0, 0, 0, 0, ClosedLoopSlot.kSlot0);
        elevatorConf.closedLoop.maxMotion.maxAcceleration(0).maxVelocity(0)
            .allowedClosedLoopError(0);
        elevatorMotor.configure(elevatorConf, ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);
    }

    private void setPosition(double position) {
        controller.setReference(position, SparkBase.ControlType.kMAXMotionPositionControl);
    }


    private void setVoltage(double v) {
        elevatorMotor.setVoltage(v);
    }

    @Override
    public void updateInputs(ElevatorInputs inputs) {
        inputs.position = Meters.of(encoder.getPosition());
    }

}
