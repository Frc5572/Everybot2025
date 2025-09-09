package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;

public record ModuleConstants(int driveMotorId, int angleMotorId, int encoderId,
    Rotation2d offset) {

}
