package frc.robot;

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
import frc.robot.subsystems.algae.AlgaeReal;
import frc.robot.subsystems.coral.CoralIntake;
import frc.robot.subsystems.coral.CoralIntakeReal;
import frc.robot.subsystems.elevator.Elevator;
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
    private final CommandXboxController pitController =
        new CommandXboxController(Constants.operatorID);

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
        pitController.povLeft().whileTrue(algae.setVoltage(() -> 0.5))
            .onFalse(algae.setVoltage(() -> 0.0));
        pitController.povRight().whileTrue(algae.setVoltage(() -> -0.5))
            .onFalse(algae.setVoltage(() -> 0.0));
        pitController.povUp().whileTrue(coralintake.wristVoltage(() -> 5))
            .onFalse(coralintake.wristVoltage(() -> 0.0));
        pitController.povDown().whileTrue(coralintake.wristVoltage(() -> -5))
            .onFalse(coralintake.wristVoltage(() -> 0.0));

        // RobotModeTriggers.autonomous().onTrue(
        // algae.moveTo(() -> -680.0).withTimeout(1.0).andThen(algae.setVoltage(() -> 0.0)));

        // driver.a().whileTrue(algae.moveTo(() -> -680.0)).onFalse(algae.setVoltage(() -> 0.0));

        double l2height = 0.0;
        double l3height = 0.85;
        double feedheight = 0.0;
        double l2angle = 225.0;
<<<<<<< HEAD
        double l3angle = 227.0;
        double transitangle = 105.0;
        double feedangle = 55.0;
=======
        double l3angle = 225.0;
        double transitangle = 105.0;
        double feedangle = 54.0;
>>>>>>> main

        double algaehome = -500;
        double algaeintaking = -785;

        Trigger eitherTrigger = driver.leftTrigger().or(driver.rightTrigger());
        Trigger eitherBumper = driver.leftBumper().or(driver.rightBumper());

        Supplier<Command> transit = () -> elevator.moveTo(() -> feedheight)
            .alongWith(coralintake.moveTo(() -> transitangle));

        driver.leftBumper()
            .whileTrue(elevator.moveTo(() -> l2height).alongWith(coralintake.moveTo(() -> l2angle)))
            .onFalse(transit.get());
        driver.rightBumper()
            .whileTrue(elevator.moveTo(() -> l3height).alongWith(coralintake.moveTo(() -> l3angle)))
            .onFalse(transit.get());
        eitherTrigger.and(eitherBumper).whileTrue(coralintake.runCoralOuttake());
        eitherTrigger.and(eitherBumper.negate()).whileTrue(elevator.moveTo(() -> feedheight)
            .alongWith(coralintake.moveTo(() -> feedangle)).alongWith(coralintake.runCoralIntake()))
            .onFalse(transit.get());
        pitController.leftTrigger().or(pitController.rightTrigger())
            .whileTrue(coralintake.runCoralOuttake());
        pitController.a().whileTrue(coralintake.runCoralIntake());

        Supplier<Command> algaeHome = () -> algae.moveTo(() -> algaehome).withTimeout(1.0)
            .andThen(algae.setVoltage(() -> 0.0));
        driver.a().whileTrue(algae.moveTo(() -> algaeintaking).alongWith(algae.runAlgaeIntake()))
            .onFalse(algaeHome.get()
                .alongWith(algae.runAlgaeIntake().withTimeout(1.0).andThen(algae.runAlgaeHold())));
        driver.b().whileTrue(algae.runAlgaeOuttake());
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
