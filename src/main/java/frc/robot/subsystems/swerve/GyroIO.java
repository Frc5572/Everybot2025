package frc.robot.subsystems.swerve;

import org.littletonrobotics.junction.AutoLog;

public interface GyroIO {

    @AutoLog
    public static class GyroInputs {
        public double yawRads;
    }

    public void updateInputs(GyroInputs inputs);

    public static class Empty implements GyroIO {

        public Empty() {}

        @Override
        public void updateInputs(GyroInputs inputs) {}
    }

}
