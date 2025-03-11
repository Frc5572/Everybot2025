package frc.robot;

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
        public static final double angleKP = 3.0;
        public static final double angleKI = 0.0;
        public static final double angleKD = 0.0;

        /* Drive Motor PID Values */

        public static final double driveKP = 0.01;
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;


        public static final double angleMinOutput = -1;
        public static final double angleMaxOutput = 1;


        public static final double trackWidth = Units.inchesToMeters(21.5);
        public static final double wheelBase = Units.inchesToMeters(23.5);
        public static final double wheelDiameter = Units.inchesToMeters(3.0);
        public static final double wheelCircumference = wheelDiameter * Math.PI;


        public static final double maxSpeed = 1.0;
        public static final double maxAngularVelocity = 9.0;

        public static NavXComType navXID = NavXComType.kMXP_SPI;
        public static final boolean invertGyro = true;


        /* Module Gear Ratios */
        public static final double driveGearRatio = (4.40 / 1.0); // X2_16
        public static final double angleGearRatio = (1.0 / 41.25); // (150 / 7) : 1

        /* Motor Inverts */
        public static final boolean angleMotorInvert = false;
        public static final boolean driveMotorInvert = true;

        /* Angle Encoder Invert */
        public static final SensorDirectionValue cancoderInvert =
            SensorDirectionValue.CounterClockwise_Positive;


        public static final SwerveDriveKinematics swerveKinematics =
            new SwerveDriveKinematics(new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /**
         * Front Left Module
         */
        public static final class Mod0 {
            public static final int angleMotorID = 1;
            public static final int driveMotorID = 2;
            public static final int canCoderID = 9;
            public static final Rotation2d angleOffset = Rotation2d.fromRotations(.355713);


        }

        /**
         * Front Right Module
         */
        public static final class Mod1 {
            public static final int angleMotorID = 3;
            public static final int driveMotorID = 4;
            public static final int canCoderID = 10;
            public static final Rotation2d angleOffset = Rotation2d.fromRotations(.310547);


        }

        /**
         * Back Left Module
         */
        public static final class Mod2 {
            public static final int angleMotorID = 5;
            public static final int driveMotorID = 6;
            public static final int canCoderID = 11;
            public static final Rotation2d angleOffset = Rotation2d.fromRotations(-.059570);


        }

        public static final class Mod3 {
            public static final int angleMotorID = 7;
            public static final int driveMotorID = 8;
            public static final int canCoderID = 12;
            public static final Rotation2d angleOffset = Rotation2d.fromRotations(.338867);


        }


    }

}
