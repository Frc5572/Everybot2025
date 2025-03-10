package frc.robot.subsystems.swerve;


import java.util.Optional;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.util.swerve.SwerveModule;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Swerve extends SubsystemBase {
    public SwerveDrivePoseEstimator swerveOdometry;
    public SwerveModule[] swerveMods;
    private final Field2d field = new Field2d();
    private double fieldOffset;
    private SwerveInputsAutoLogged inputs = new SwerveInputsAutoLogged();
    private SwerveIO swerveIO;
    private boolean hasInitialized = false;

    public Swerve(SwerveIO swerveIO) {
        this.swerveIO = swerveIO;
        fieldOffset = getGyroYaw().getDegrees();
        swerveMods = new SwerveModule[] {
            swerveIO.createSwerveModule(0, Constants.Swerve.Mod0.driveMotorID,
                Constants.Swerve.Mod0.angleMotorID, Constants.Swerve.Mod0.canCoderID,
                Constants.Swerve.Mod0.angleOffset),
            swerveIO.createSwerveModule(1, Constants.Swerve.Mod1.driveMotorID,
                Constants.Swerve.Mod1.angleMotorID, Constants.Swerve.Mod1.canCoderID,
                Constants.Swerve.Mod1.angleOffset),
            swerveIO.createSwerveModule(2, Constants.Swerve.Mod2.driveMotorID,
                Constants.Swerve.Mod2.angleMotorID, Constants.Swerve.Mod2.canCoderID,
                Constants.Swerve.Mod2.angleOffset),
            swerveIO.createSwerveModule(3, Constants.Swerve.Mod3.driveMotorID,
                Constants.Swerve.Mod3.angleMotorID, Constants.Swerve.Mod3.canCoderID,
                Constants.Swerve.Mod3.angleOffset)};
        swerveOdometry = new SwerveDrivePoseEstimator(Constants.Swerve.swerveKinematics,
            getGyroYaw(), getModulePositions(), new Pose2d());

        swerveIO.updateInputs(inputs);

        RobotContainer.mainDriverTab.add("Field Pos", field).withWidget(BuiltInWidgets.kField)
            .withSize(0, 0).withPosition(0, 0);
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative) {
        ChassisSpeeds chassisSpeeds = fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(translation.getX(), translation.getY(),
                rotation, getFieldRelativeHeading())
            : new ChassisSpeeds(translation.getX(), translation.getY(), rotation);

        setModuleStates(chassisSpeeds);
    }

    public Command teleOPDrive(CommandXboxController controller) {
        return this.run(() -> {
            double yaxis = -controller.getLeftY();
            double xaxis = -controller.getLeftX();
            double raxis = -controller.getRightX();
            /* Deadbands */
            yaxis = MathUtil.applyDeadband(yaxis, 0.1);
            xaxis = MathUtil.applyDeadband(xaxis, 0.1);
            xaxis *= xaxis * Math.signum(xaxis);
            yaxis *= yaxis * Math.signum(yaxis);
            raxis = (Math.abs(raxis) < Constants.STICK_DEADBAND) ? 0 : raxis;
            Translation2d translation =
                new Translation2d(yaxis, xaxis).times(Constants.Swerve.maxSpeed);
            double rotation = raxis * Constants.Swerve.maxAngularVelocity;
            this.drive(translation, rotation, true);
        });
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);;
        Logger.recordOutput("Swerve/DesiredStates", desiredStates);
        for (SwerveModule mod : swerveMods) {
            mod.setDesiredState(desiredStates[mod.moduleNum], false);
        }
    }

    public void setModuleStates(ChassisSpeeds chassisSpeeds) {
        ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(chassisSpeeds, 0.02);
        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(targetSpeeds);
        Logger.recordOutput("Swerve/Desired Module States", swerveModuleStates);
        setModuleStates(swerveModuleStates);
    }

    public ChassisSpeeds getChassisSpeeds() {
        return Constants.Swerve.swerveKinematics.toChassisSpeeds(getModuleStates());
    }

    @AutoLogOutput(key = "Swerve/Module States")
    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[swerveMods.length];
        for (SwerveModule mod : swerveMods) {
            states[mod.moduleNum] = mod.getState();
        }
        return states;
    }

    @AutoLogOutput(key = "Swerve/Absolute Module States")
    public SwerveModuleState[] getAbsoluteModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[swerveMods.length];
        for (SwerveModule mod : swerveMods) {
            states[mod.moduleNum] =
                new SwerveModuleState(mod.getState().speedMetersPerSecond, mod.getCANcoder());
        }
        return states;
    }


    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[swerveMods.length];
        for (SwerveModule mod : swerveMods) {
            positions[mod.moduleNum] = mod.getPosition();
        }
        return positions;
    }

    @AutoLogOutput(key = "Odometry/Robot")
    public Pose2d getPose() {
        return swerveOdometry.getEstimatedPosition();
    }

    public void resetOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), pose);
    }

    public Rotation2d getHeading() {
        return getPose().getRotation();
    }

    public Rotation2d getGyroYaw() {
        float yaw = inputs.yaw;
        return (Constants.Swerve.invertGyro) ? Rotation2d.fromDegrees(-yaw)
            : Rotation2d.fromDegrees(yaw);
    }

    public Rotation2d getFieldRelativeHeading() {
        return Rotation2d.fromDegrees(getGyroYaw().getDegrees() - fieldOffset);
    }

    public void resetFieldRelativeOffset() {
        // gyro.zeroYaw();
        fieldOffset = getGyroYaw().getDegrees() + 180;
    }

    public void resetModulesToAbsolute() {
        for (SwerveModule mod : swerveMods) {
            mod.resetToAbsolute();
        }
    }

    public void resetPvInitialization() {
        hasInitialized = false;
    }

    @Override
    public void periodic() {
        swerveIO.updateInputs(inputs);
        for (var mod : swerveMods) {
            mod.periodic();
        }
        swerveOdometry.update(getGyroYaw(), getModulePositions());
        Logger.processInputs("Swerve", inputs);


        Logger.recordOutput("/Swerve/hasInitialized", hasInitialized);

        SmartDashboard.putBoolean("Has Initialized", hasInitialized);
        SmartDashboard.putNumber("Gyro Yaw", getGyroYaw().getDegrees());
    }

    public void setMotorsZero() {
        System.out.println("Setting Zero!!!!!!");
        setModuleStates(new ChassisSpeeds(0, 0, 0));
    }

    public void wheelsIn() {
        swerveMods[0].setDesiredState(new SwerveModuleState(2, Rotation2d.fromDegrees(45)), false);
        swerveMods[1].setDesiredState(new SwerveModuleState(2, Rotation2d.fromDegrees(135)), false);
        swerveMods[2].setDesiredState(new SwerveModuleState(2, Rotation2d.fromDegrees(-45)), false);
        swerveMods[3].setDesiredState(new SwerveModuleState(2, Rotation2d.fromDegrees(-135)),
            false);
        this.setMotorsZero();
    }

    public SwerveModulePosition[] getSwerveModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[swerveMods.length];
        for (SwerveModule mod : swerveMods) {
            positions[mod.moduleNum] = mod.getPosition();
        }
        return positions;
    }

    public static boolean shouldFlipPath() {
        Optional<Alliance> ally = DriverStation.getAlliance();
        if (ally.isPresent()) {
            return ally.get() == Alliance.Red;
        }
        return false;
    }

}
