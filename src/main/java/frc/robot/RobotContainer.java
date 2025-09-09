package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Robot.RobotRunType;
import frc.robot.subsystems.algae.Algae;
import frc.robot.subsystems.algae.AlgaeIO;
import frc.robot.subsystems.algae.AlgaeReal;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorReal;
import frc.robot.subsystems.swerve.GyroIO;
import frc.robot.subsystems.swerve.GyroNavX;
import frc.robot.subsystems.swerve.ModuleIO;
import frc.robot.subsystems.swerve.ModuleSparkMax;
import frc.robot.subsystems.swerve.Swerve;

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
                swerve = new Swerve(new GyroNavX(), ModuleSparkMax::new);
                elevator = new Elevator(new ElevatorReal());
                algae = new Algae(new AlgaeReal());
                break;
            case kSimulation:
                // drivetrain = new Drivetrain(new DrivetrainSim() {});

                break;
            default:
                swerve = new Swerve(new GyroIO.Empty(), ModuleIO.Empty::new);
                elevator = new Elevator(new ElevatorIO() {});
                algae = new Algae(new AlgaeIO() {});
        }
        swerve.setDefaultCommand(
            swerve.teleopDrive(driver::getLeftX, driver::getLeftY, driver::getRightX));
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
    }

    private void setupDriver() {
        driver.y().onTrue(swerve.resetFieldOrientation());
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
