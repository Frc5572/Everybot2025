package frc.robot;

/**
 * Constants file.
 */
public final class Constants {
    /**
     * Stick Deadband
     */
    public static final double stickDeadband = 0.1;
    /**
     * Driver ID
     */
    public static final int driverID = 0;
    /**
     * Operator ID
     */
    public static final int operatorID = 1;

    /**
     * Motor CAN id's.
     */
    public static final class Motors {
    }

    /**
     * Pneumatics CAN id constants.
     */
    public static final class Pneumatics {
    }

    public static final class CoralIntakeConstants {
        public final int kElevatorMotorCanId = 0;
        public final int kArmMotorCanId = 0;
        public final int kIntakeMotorCanId = 0;
    }

    public static final class ElevatorSetpoints {
        public static final int kFeederStation = 0;
        public static final int kLevel1 = 0;
        public static final int kLevel2 = 0;
        public static final int kLevel3 = 0;
        public static final int kLevel4 = 0;
    }

    public static final class ArmSetpoints {
        public static final double kFeederStation = 0;
        public static final double kLevel1 = 0;
        public static final double kLevel2 = 0;
        public static final double kLevel3 = 0;
        public static final double kLevel4 = 0;
    }

    public static final class IntakeSetpoints {
        public static final double kForward = 0;
        public static final double kReverse = 0;
    }

}
