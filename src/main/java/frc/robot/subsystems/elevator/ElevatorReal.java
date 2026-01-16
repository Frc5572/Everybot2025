package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;

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
        elevatorConf.closedLoop.pid(100, 0, 0, ClosedLoopSlot.kSlot0);
        elevatorMotor.configure(elevatorConf, ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);
    }

    @Override
    public void setPosition(double position, double ff) {
        controller.setSetpoint(position, ControlType.kPosition, ClosedLoopSlot.kSlot0, ff);

    }

    @Override
    public void setVoltage(double v) {
        elevatorMotor.setVoltage(v);
    }

    @Override
    public void updateInputs(ElevatorInputs inputs) {
        inputs.voltage = Volts.of(elevatorMotor.getBusVoltage());
        inputs.position = Meters.of(encoder.getPosition());
        inputs.rotation = Rotations.of(encoder.getPosition());
    }

    @Override
    public void setPID(double kP, double kI, double kD, double kFF) {
        elevatorConf.closedLoop.pid(kP, kI, kD, ClosedLoopSlot.kSlot0);
        elevatorMotor.configure(elevatorConf, ResetMode.kNoResetSafeParameters,
            PersistMode.kNoPersistParameters);
    }
}
