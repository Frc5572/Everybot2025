package frc.robot.subsystems.Elevator;

import com.revrobotics.spark.SparkMax;

public class ElevatorReal implements ElevatorIO{
    private final SparkMax ElevatorMotor = new SparkMax(ElevatorConstants.kMotorPort, MotorType.kBrushless);
    private final Encoder ElevatorEncoder = new Encoder(ElevatorConstants.kEncoderChannelA, ElevatorConstants.kEncoderChannelB);

    public Elevator




}
