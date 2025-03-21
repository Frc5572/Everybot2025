package frc.robot.subsystems.swerve;

import org.littletonrobotics.junction.AutoLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Quaternion;

public interface questNavIO {
    @AutoLog
    public static class QuestNavInputs {
        public double questBattery;
        public Pose2d questPose;
        public Quaternion questQuaternion;
        public boolean questConnected;
        public float questYaw;
    }

    public default void updateInputs(QuestNavInputs inputs) {}

    public default void zeroPosition() {}

    public default void zeroHeading() {}
}
