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

    public static final class CoralSubsystemConstants {
        public static final int kWristMotorCanId = 0;
        public static final int kIntakeMotorCanId = 0;
        public static final double INTAKE_VOLTAGE = 0.0;
        public static final double OUTTAKE_VOLTAGE = 0.0;

    }



    public static final class IntakeSetpoints {
        public static final double kForward = 0;
        public static final double kReverse = 0;
    }

}
