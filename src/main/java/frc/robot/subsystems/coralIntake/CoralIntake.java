package frc.robot.subsystems.coralIntake;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;


public class CoralIntake extends SubsystemBase {
    private CoralIntakeIO io;
    private CoralIntakeInputsAutoLogged inputs = new CoralIntakeInputsAutoLogged();
    private CommandXboxController operator;


    ProfiledPIDController wristPIDController = new ProfiledPIDController(
        Constants.CoralSubsystem.WristPID_KP, Constants.CoralSubsystem.WristPID_KI,
        Constants.CoralSubsystem.WristPID_KD, new TrapezoidProfile.Constraints(0, 0));

    private double estimatedWristAngle = 0;


    public CoralIntake(CoralIntakeIO io) {
        this.io = io;
        io.updateInputs(inputs);
        estimatedWristAngle = getWristAngle().getRotations();
        wristPIDController.setTolerance(Rotation2d.fromDegrees(0).getRotations());
        wristPIDController.setIZone(Rotation2d.fromDegrees(0).getRotations());



    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Coral", inputs);
        wristPIDController.setGoal(Constants.CoralSubsystem.AMP_ANGLE.getRotations());

        Rotation2d wristAngle = getWristAngle();

        double wristPIDValue = wristPIDController.calculate(wristAngle.getRotations());



    }

    public void setCoralVoltage(double voltage) {
        Logger.recordOutput("Coral/Voltage", voltage);
        io.setCoralVoltage(voltage);
    }


    public Rotation2d getWristAngle() {
        return Rotation2d.fromRotations(estimatedWristAngle);
    }

    public void setWristAngle(Rotation2d angle) {
        wristPIDController.setGoal(angle.getRotations());
    }

    public Command runCoralIntake() {
        return Commands.runEnd(() -> setCoralVoltage(6), () -> setCoralVoltage(0));
    }

    public Command runCoralOuttake() {
        return Commands.runEnd(() -> setCoralVoltage(-6), () -> setCoralVoltage(0));
    }


    public Command runCoralWristManually(CommandXboxController controller) {

    }

}
