package frc.robot.subsystems.SwerveModule;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule {
    // private final CANSparkMax driveMotor;
    // private final CANSparkMax turnMotor;

    // private final CANEncoder driveEncoder;
    // private final CANEncoder turnEncoder;

    // Talon leftFrontTalon = null;
    // Talon leftBackTalon = null;
    // Talon rightFrontTalon = null;
    // Talon rightBackTalon = null;
    private final Talon driveMotor;
    private final SparkMax turnMotor;

    // private final
    private final Encoder
    private final CANEncoder turnEncoder;



    private final PIDController turnPidController;

    private final AnaLogInput absoluteEncoder;
    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffsetRad;

    public SwerveModule(int driveMotorId, int turnMotorId, boolean driveMotorReversed,
        boolean turnMotorReversed, int absoluteEncoderId, double absoluteEncoderOffset,
        boolean absoluteEncoderReversed) {
        this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
        this.absoluteEncoderReversed = absoluteEncoderReversed;
        absoluteEncoder = new AnalogInput(absoluteEncoderId);

        // driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
        // turnMotor = new CANSparkMax(turnMotorId, MotorType.kBrushless);
        driveMotor = new TalonFX(driveMotorId);
        turnMotorId = new SparkMax(turnMotorId, MotorType.kBrushless);

        driveMotor.setInverted(driveMotorReversed);
        turnMotor.setInverted(turnMotorReversed);


        turnEncoder = turnMotor.getEncoder();

        turnEncoder.setPositionConversionFactor(ModuleConstants.kTurnEncoderRot2Rad);
        turnEncoder.setVelocityConversionFactor(ModuleConstants.kTurnEncoderRPM2RadPerSec);

        turnPidController = new PIDController(ModuleConstants.kPTurn, 0, 0);




    }



}
