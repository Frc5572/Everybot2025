package frc.robot.subsystems.swerve;


import java.util.function.DoubleSupplier;
import java.util.function.IntFunction;
import java.util.stream.IntStream;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Swerve
 */
public class Swerve extends SubsystemBase {

    public final GyroIO gyroIO;
    public final Module[] modules;
    private final String[] moduleStrings;
    private static final String[] moduleOrder =
        new String[] {"FrontLeft", "FrontRight", "BackLeft", "BackRight"};

    private final GyroInputsAutoLogged gyroInputs = new GyroInputsAutoLogged();

    public Swerve(GyroIO io, IntFunction<ModuleIO> moduleFunc) {
        this.gyroIO = io;
        this.modules = IntStream.range(0, 4).mapToObj(i -> new Module(moduleFunc.apply(i)))
            .toArray(Module[]::new);
        this.moduleStrings = IntStream.range(0, 4).mapToObj(i -> "Swerve/Modules/" + moduleOrder[i])
            .toArray(String[]::new);
    }

    private void updateInputs() {
        gyroIO.updateInputs(gyroInputs);
        Logger.processInputs("Swerve/Gyro", gyroInputs);
        for (int i = 0; i < 4; i++) {
            modules[i].updateInputs(moduleStrings[i]);
        }
    }

    @Override
    public void periodic() {
        updateInputs();
    }

    public Command teleopDrive(DoubleSupplier x, DoubleSupplier y, DoubleSupplier r) {
        return this.run(() -> {

        });
    }

    public Command resetFieldOrientation() {
        return this.runOnce(() -> {

        });
    }

}
