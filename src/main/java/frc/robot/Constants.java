package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.studica.frc.AHRS.NavXComType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * Constants file.
 */
public final class Constants {
    /**
     * Stick Deadband
     */
    public static final double STICK_DEADBAND = 0.1;
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

    public static final class Swerve {



        /* Angle Motor PID Values */
        public static final double angleKP = 5.0;
        public static final double angleKI = 0.0;
        public static final double angleKD = 0.0;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.0;
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;


        public static final double angleMinOutput = -1;
        public static final double angleMaxOutput = 1;


        /* Module Gear Ratios */
        public static final double driveGearRatio = (4.40 / 1.0); // X2_16
        public static final double angleGearRatio = (1.0 / 41.25); // (150 / 7) : 1

        /* Motor Inverts */
        public static final InvertedValue angleMotorInvert = InvertedValue.Clockwise_Positive;
        public static final InvertedValue driveMotorInvert =
            InvertedValue.CounterClockwise_Positive;

        /* Angle Encoder Invert */
        public static final SensorDirectionValue cancoderInvert =
            SensorDirectionValue.CounterClockwise_Positive;


        public static final double trackWidth = Units.inchesToMeters(24.0);
        public static final double wheelBase = Units.inchesToMeters(24.0);
        public static final double wheelDiameter = Units.inchesToMeters(3.8);
        public static final double wheelCircumference = wheelDiameter * Math.PI;
        public static final double maxSpeed = 0.0;
        public static final double maxAngularVelocity = 0.0;

        public static NavXComType navXID = NavXComType.kMXP_SPI;
        public static final boolean invertGyro = true;


        public static final SwerveDriveKinematics swerveKinematics =
            new SwerveDriveKinematics(new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        public static final class Mod0 {
            public static final int driveMotorID = 0;
            public static final int angleMotorID = 0;
            public static final int canCoderID = 0;
            public static final Rotation2d angleOffset = null;


        }

        public static final class Mod1 {
            public static final int driveMotorID = 0;
            public static final int angleMotorID = 0;
            public static final int canCoderID = 0;
            public static final Rotation2d angleOffset = null;


        }

        public static final class Mod2 {
            public static final int driveMotorID = 0;
            public static final int angleMotorID = 0;
            public static final int canCoderID = 0;
            public static final Rotation2d angleOffset = null;


        }

        public static final class Mod3 {
            public static final int driveMotorID = 0;
            public static final int angleMotorID = 0;
            public static final int canCoderID = 0;
            public static final Rotation2d angleOffset = null;


        }



    }

}
