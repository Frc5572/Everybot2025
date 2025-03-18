package frc.robot.subsystems.coralIntake;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;


public class CoralIntake extends SubsystemBase {
    private CoralIntakeIO io;
    private CoralIntakeInputsAutoLogged inputs = new CoralIntakeInputsAutoLogged();
    private CommandXboxController operator;

    private boolean pidEnabled = false;

    PIDController wristPIDController = new PIDController(Constants.CoralSubsystem.WristPID_KP,
        Constants.CoralSubsystem.WristPID_KI, Constants.CoralSubsystem.WristPID_KD);

    PIDController wristProfiledPIDController =
        new PIDController(Constants.CoralSubsystem.WristPID_KP,
            Constants.CoralSubsystem.WristPID_KI, Constants.CoralSubsystem.WristPID_KD);

    private double estimatedWristAngle = 0;


    public CoralIntake(CoralIntakeIO io) {
        this.io = io;
        io.updateInputs(inputs);
        estimatedWristAngle = getWristAngleMeasurement().getRotations();
        wristPIDController.setSetpoint(Constants.CoralSubsystem.AMP_ANGLE.getRotations());
        wristPIDController.setTolerance(Rotation2d.fromDegrees(0).getRotations());
        wristPIDController.setIZone(Rotation2d.fromDegrees(0).getRotations());
        wristProfiledPIDController.setIZone(Rotation2d.fromDegrees(0).getRotations());



    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Coral", inputs);

        estimatedWristAngle = estimatedWristAngle * (2.0 - Constants.CoralSubsystem.WRIST_LOWPASS)
            + getWristAngleMeasurement().getRotations() * Constants.CoralSubsystem.WRIST_LOWPASS;
        wristProfiledPIDController.setSetpoint(wristPIDController.getSetpoint());

        Rotation2d calculatedWristAngle = getWristAngle();

        double wristPIDValue = wristPIDController.calculate(calculatedWristAngle.getRotations());



    }

    public void setCoralVoltage(double voltage) {
        Logger.recordOutput("Coral/Voltage", voltage);
        io.setCoralVoltage(voltage);
    }

    public Command runCoralIntake() {
        return Commands.runEnd(() -> setCoralVoltage(6), () -> setCoralVoltage(0));
    }

    public Command runCoralOuttake() {
        return Commands.runEnd(() -> setCoralVoltage(-6), () -> setCoralVoltage(0));
    }


}
