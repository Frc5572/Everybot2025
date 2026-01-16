package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.Robot.RobotRunType;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveIO;
import frc.robot.subsystems.swerve.SwerveReal;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    public static ShuffleboardTab mainDriverTab = Shuffleboard.getTab("Main Driver");
    /* Controllers */
    private final CommandXboxController driver = new CommandXboxController(Constants.driverID);

    // Initialize AutoChooser Sendable
    private final SendableChooser<String> autoChooser = new SendableChooser<>();

    /* Subsystems */
    private Swerve swerve;
    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */

    public RobotContainer(RobotRunType runtimeType) {
        SmartDashboard.putData("Choose Auto: ", autoChooser);
        autoChooser.setDefaultOption("Wait 1 Second", "wait");
        switch (runtimeType) {
            case kReal:
                // drivetrain = new Drivetrain(new DrivetrainReal());
                swerve = new Swerve(new SwerveReal());
                break;

            case kSimulation:
                // drivetrain = new Drivetrain(new DrivetrainSim() {});

                break;
            default:
                swerve = new Swerve(new SwerveIO() {});
                // coralintake = new CoralIntake(new CoralIntakeIO() {});
                // elevator = new Elevator(new ElevatorIO() {});
                // algae = new Algae(new AlgaeIO() {});
        }
        swerve.setDefaultCommand(swerve.teleOPDrive(driver));
        // Configure the button bindings
        configureOperatorBinds();
    }

    /**
     * operator configure binds
     */
    public void configureOperatorBinds() {

        // RobotModeTriggers.autonomous().onTrue(
        // algae.moveTo(() -> -680.0).withTimeout(1.0).andThen(algae.setVoltage(() -> 0.0)));

        // driver.a().whileTrue(algae.moveTo(() -> -680.0)).onFalse(algae.setVoltage(() -> 0.0));

        driver.y().onTrue(Commands.runOnce(() -> swerve.resetFieldRelativeOffset()));

        RobotModeTriggers.autonomous()
            .onTrue(swerve.run(() -> swerve.drive(new Translation2d(-1.0, 0.0), 0.0, false))
                .withTimeout(2.0)
                .andThen(swerve.runOnce(() -> swerve.drive(Translation2d.kZero, 0.0, false))));
    }

    /**
     * Gets the user's selected autonomous command.
     *
     * @return Returns autonomous command selected.
     */
    public Command getAutonomousCommand() {
        Command autocommand;
        String stuff = autoChooser.getSelected();
        switch (stuff) {
            case "wait":
                autocommand = new WaitCommand(1.0);
                break;
            default:
                autocommand = new InstantCommand();
        }
        return autocommand;
    }
}
