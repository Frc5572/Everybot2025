package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;
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
        encoder.setPosition(0.0);
        elevatorConf.encoder
            .positionConversionFactor(Inches.of(60).in(Meters) / Rotations.of(190).in(Rotations));
        elevatorConf.closedLoop.pidf(100, 0, 0, 0.22, ClosedLoopSlot.kSlot0);
        elevatorConf.closedLoop.maxMotion.maxAcceleration(0).maxVelocity(0)
            .allowedClosedLoopError(0);
        elevatorMotor.configure(elevatorConf, ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);
    }

    public void setPosition(double position) {
        controller.setReference(position, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }


    public void setVoltage(double v) {
        elevatorMotor.setVoltage(v);
    }

    @Override
    public void updateInputs(ElevatorInputs inputs) {
        inputs.voltage = Volts.of(elevatorMotor.getBusVoltage());
        inputs.position = Meters.of(encoder.getPosition());
        inputs.rotation = Rotations.of(encoder.getPosition());
    }

    //
    // @Override
    // public void setPosition(double position) {
    // controller.setReference(position, SparkBase.ControlType.kMAXMotionPositionControl);
    // }


    // @Override
    // public void setVoltage(double v) {
    // elevatorMotor.setVoltage(v);
    // }



}
