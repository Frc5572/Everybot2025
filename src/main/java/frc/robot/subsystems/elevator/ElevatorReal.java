package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

/** Elevator Real layer */
public class ElevatorReal implements ElevatorIO {
    private final SparkMax elevatorMotor = new SparkMax(10, MotorType.kBrushless);
    private final SparkBaseConfig elevatorConf = new SparkMaxConfig();
    private final SparkClosedLoopController controller = elevatorMotor.getClosedLoopController();
    private final RelativeEncoder encoder = elevatorMotor.getEncoder();

    public ElevatorReal() {
        config();
    }



    private void config() {
        elevatorConf.inverted(false);
        // elevatorConf.encoder.positionConversionFactor(1.0 / 25.0);
        elevatorConf.closedLoop.pidf(5, 0, 0, 0, ClosedLoopSlot.kSlot0);
        elevatorConf.closedLoop.maxMotion.maxAcceleration(0).maxVelocity(0)
            .allowedClosedLoopError(0);
        elevatorMotor.configure(elevatorConf, ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);
    }

    public void setPosition(double position) {
        controller.setReference(position, ControlType.kPosition);
    }


    public void setVoltage(double v) {
        elevatorMotor.setVoltage(v);
    }

    @Override
    public void updateInputs(ElevatorInputs inputs) {
        inputs.position = Meters.of(encoder.getPosition());
        inputs.rotation = Rotations.of(encoder.getPosition());
    }

    @Override
    public void setPosition(double position) {
        controller.setReference(position, SparkBase.ControlType.kMAXMotionPositionControl);
    }


    @Override
    public void setVoltage(double v) {
        elevatorMotor.setVoltage(v);
    }


    @Override
    public void setPosition(double position) {
        controller.setReference(position, SparkBase.ControlType.kMAXMotionPositionControl);
    }


    @Override
    public void setVoltage(double v) {
        elevatorMotor.setVoltage(v);
    }


}
