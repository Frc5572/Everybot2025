package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.Swerve;

public class TeleopSwerve extends Command {

    private boolean fieldRelative;
    private Swerve swerveDrive;
    private CommandXboxController controller;
    private double speedMultiplier = 1;

    public TeleopSwerve(Swerve swerveDrive, CommandXboxController controller,
        boolean fieldRelative) {
        this.swerveDrive = swerveDrive;
        addRequirements(swerveDrive);
        this.fieldRelative = fieldRelative;
        this.controller = controller;
    }

    public TeleopSwerve(Swerve swerveDrive, CommandXboxController controller, boolean fieldRelative,
        double speedMultiplier) {
        this(swerveDrive, controller, fieldRelative);
        this.speedMultiplier = speedMultiplier;

    }

    @Override
    public void execute() {
        double yaxis = -controller.getLeftY() * speedMultiplier;
        double xaxis = -controller.getLeftX() * speedMultiplier;
        double raxis = -controller.getRightX() * speedMultiplier;


        yaxis = (Math.abs(yaxis) < Constants.STICK_DEADBAND) ? 0
            : (yaxis - Constants.STICK_DEADBAND) / (1.0 - Constants.STICK_DEADBAND);
        xaxis = (Math.abs(xaxis) < Constants.STICK_DEADBAND) ? 0
            : (yaxis - Constants.STICK_DEADBAND) / (1.0 - Constants.STICK_DEADBAND);
        xaxis *= xaxis * Math.signum(xaxis);
        yaxis *= yaxis * Math.signum(yaxis);
        raxis = (Math.abs(raxis) < Constants.STICK_DEADBAND) ? 0 : raxis;


        Translation2d translation =
            new Translation2d(yaxis, xaxis).times(Constants.Swerve.maxSpeed);
        double rotation = raxis * Constants.Swerve.maxAngularVelocity;
        swerveDrive.drive(translation, rotation, fieldRelative);


    }



}
