package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Robot.RobotRunType;
import frc.robot.subsystems.algae.Algae;
import frc.robot.subsystems.algae.AlgaeIO;
import frc.robot.subsystems.algae.AlgaeReal;
import frc.robot.subsystems.coral.CoralIntake;
import frc.robot.subsystems.coral.CoralIntakeIO;
import frc.robot.subsystems.coral.CoralIntakeReal;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorReal;
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
    private final CommandXboxController operator = new CommandXboxController(Constants.operatorID);

    // Initialize AutoChooser Sendable
    private final SendableChooser<String> autoChooser = new SendableChooser<>();

    /* Subsystems */
    private Swerve swerve;
    private CoralIntake coralintake;
    private Elevator elevator;
    private Algae algae;

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
                coralintake = new CoralIntake(new CoralIntakeReal());
                elevator = new Elevator(new ElevatorReal());
                algae = new Algae(new AlgaeReal());
                break;

            case kSimulation:
                // drivetrain = new Drivetrain(new DrivetrainSim() {});

                break;
            default:
                swerve = new Swerve(new SwerveIO() {});
                coralintake = new CoralIntake(new CoralIntakeIO() {});
                elevator = new Elevator(new ElevatorIO() {});
                algae = new Algae(new AlgaeIO() {});
        }
        swerve.setDefaultCommand(swerve.teleOPDrive(driver));
        // Configure the button bindings
        configureButtonBindings();
        setupDriver();
        configureOperatorBinds();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses
     * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
     * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        SmartDashboard.putNumber("height", 40);
        SmartDashboard.putNumber("voltage", 0);
    }

    /**
     * operator configure binds
     */
    public void configureOperatorBinds() {
        operator.x().whileTrue(algae.runAlgaeIntake());
        operator.y().whileTrue(algae.runAlgaeOuttake());
        operator.a().whileTrue(algae.algaeWristDown());
        operator.b().whileTrue(algae.algaeWristUp());

        operator.povUp().whileTrue(elevator.setVoltage(() -> 5));
        operator.povDown().whileTrue(elevator.setVoltage(() -> -5));

        operator.leftTrigger().whileTrue(coralintake.runCoralIntake());
        operator.rightTrigger().whileTrue(coralintake.runCoralOuttake());
        operator.rightBumper().whileTrue(coralintake.wristUp());
        operator.leftBumper().whileTrue(coralintake.wristDown());
    }

    private void setupDriver() {
        driver.y().onTrue(Commands.runOnce(() -> swerve.resetFieldRelativeOffset()));
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
