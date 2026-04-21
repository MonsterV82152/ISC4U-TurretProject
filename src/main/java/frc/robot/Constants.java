package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public final class Constants {
    enum Mode {
        REAL, SIM, REPLAY
    }

    public static final Mode ROBOT_MODE = Robot.isReal() ? Mode.REAL : Mode.SIM;
    public static final boolean IS_SIM = ROBOT_MODE == Mode.SIM;
    public static final String LOGGING_PATH = "/home/lvuser/logs";
    public static final double LOOP_PERIOD = 0.02;

    public final class TurretConstants {
        public static final double SWIVEL_KP = 0.1;
        public static final double SWIVEL_KI = 0.0;
        public static final double SWIVEL_KD = 0.0;

        public static final double SIM_SWIVEL_KP = 50.0;
        public static final double SIM_SWIVEL_KI = 0.0;
        public static final double SIM_SWIVEL_KD = 0.0;

        public static final Rotation2d SWIVEL_TOLERANCE = Rotation2d.fromDegrees(5);

        public static final double FLYWHEEL_KP = 0.1;
        public static final double FLYWHEEL_KI = 0.0;
        public static final double FLYWHEEL_KD = 0.0;
        public static final double FLYWHEEL_KS = 0.0;
        public static final double FLYWHEEL_KV = 0.0;

        public static final double SIM_FLYWHEEL_KP = 0.0;
        public static final double SIM_FLYWHEEL_KI = 0.0;
        public static final double SIM_FLYWHEEL_KD = 0.0;
        public static final double SIM_FLYWHEEL_KS = 0.0;
        public static final double SIM_FLYWHEEL_KV = 0.13;

        public static final double FLYWHEEL_RPS_TOLERANCE = 0.1;

        public static final double MIN_DISTANCE = 1;
        public static final double MAX_DISTANCE = 25;

        // Encoder Constants (CTRE Mag Encoder = 4096 ticks per revolution)
        public static final double ENCODER_TICKS_PER_REV = 4096.0;
        public static final double SWIVEL_GEAR_RATIO = 1.0; // TODO: Set actual gear ratio
        public static final double HOOD_GEAR_RATIO = 1.0; // TODO: Set actual gear ratio

        // Conversion factors: ticks to degrees
        public static final double SWIVEL_TICKS_TO_DEGREES = 360.0 / ENCODER_TICKS_PER_REV;
        public static final double HOOD_TICKS_TO_DEGREES = 360.0 / ENCODER_TICKS_PER_REV;

        // Swivel Constants
        public static final int SWIVEL_MOTOR_ID = 8; // TalonSRX with 17-tooth encoder
        public static final int SWIVEL_ENCODER_ID = 5; // TalonSRX with 11-tooth encoder

        public static final double SWIVEL_P = 0.1;
        public static final double SWIVEL_I = 0.0;
        public static final double SWIVEL_D = 0.0;

        public static final double SWIVEL_CURRENT_LIMIT = 0.0;
        public static final double SWIVEL_MIN_ANGLE = 0;
        public static final double SWIVEL_MAX_ANGLE = 405;
        // Hood Constants
        public static final int HOOD_MOTOR_ID = 11;
        public static final int HOOD_CANCODER_ID = -1;

        public static final double HOOD_P = 1.0;
        public static final double HOOD_I = 0.0;
        public static final double HOOD_D = 0.0;

        public static final double HOOD_CURRENT_LIMIT = 0.0;
        public static final double HOOD_MIN_ANGLE = 30;
        public static final double HOOD_MAX_ANGLE = 50;
        // Shoot Constants
        public static final int SHOOT_MOTOR_ID = 3;

        public static final double SHOOT_P = 0.000025;
        public static final double SHOOT_I = 0;
        public static final double SHOOT_D = 0;
        public static final double SHOOT_V = 0.000082;

        public static final double SHOOT_CURRENT_LIMIT = 0;
        public static final double SHOOT_MAX_RPS = 80;
        public static final double SHOOT_MIN_RPS = 20;
        public static final double SHOOT_WHEEL_RADIUS = 0.0508; // meters

        // Physical Turret Constants
        public static final Translation2d TURRET_A_POSITION = new Translation2d(2.0, 2.0); // meters
        public static final Translation2d TURRET_B_POSITION = new Translation2d(3.0, 4.0); // meters

        public static final double METAL_PLATES_FLYWHEEL_WEIGHT_KG = Units.lbsToKilograms(0.2);
        public static final double FLYWHEEL_FLYWHEEL_WEIGHT_KG = Units.lbsToKilograms(0.2);
        public static final double FLYWHEEL_RADIUS_METERS = Units.inchesToMeters(1.5); // 1.5 inches in meters
        public static final double METAL_PLATE_RADIUS_METERS = FLYWHEEL_RADIUS_METERS - 0.01;
        // public static final double FLYWHEEL_MOMENT_OF_INERTIA =
        // FLYWHEEL_FLYWHEEL_WEIGHT_KG * FLYWHEEL_RADIUS_METERS * FLYWHEEL_RADIUS_METERS
        // / 2.0 + 2 * METAL_PLATES_FLYWHEEL_WEIGHT_KG * METAL_PLATE_RADIUS_METERS *
        // METAL_PLATE_RADIUS_METERS;
        public static final double FLYWHEEL_MOMENT_OF_INERTIA = 0.0001656946 * 2;
        public static final double SWIVEL_MOMENT_OF_INERTIA = 0.00395063532;

    }

    public final class ShooterUtilConstants {
        public static final double GEAR_1_TOOTH_COUNT = 11;
        public static final double GEAR_2_TOOTH_COUNT = 17;
        public static final double GEAR_0_TOOTH_COUNT = 120;
        public static final double DIFFERENCE_PER_ROTATION = 360 - (360 / GEAR_1_TOOTH_COUNT) * GEAR_2_TOOTH_COUNT;

        public static final int MOD_INVERSE_A_MOD_B = 14;
        public static final double GRAVITY = -9.81; // m/s^2
        public static final double MIN_SHOT_TIME = 0.84837869189; // seconds
        public static final double MIN_Y_VEL = 6.31411118052;
        public static final double MIN_NEGATIVE_Y_VELOCITY = -5.0; // m/s
        public static final double LATENCY_OFFSET = 0.2; // seconds
        public static final int MEDIAN_FILTER_SIZE = 5; // Window size for median filter
        public static final int MOVING_AVG_TAPS = 3; // Number of samples for moving average

        public static final double SHOOTER_SIM_CONSTANT = 0.4;
    }

    public final class BallSimConstants {
        // Ball physical properties
        public static final double BALL_MASS_KG = Units.lbsToKilograms(0.5); // Mass of a typical game ball (e.g., cargo
                                                                             // ball)
        public static final double BALL_RADIUS_METERS = Units.inchesToMeters(2.9); // Radius of the ball

        // Simulation properties
        public static final double BALL_LIFETIME_SECONDS = 5.0; // How long a ball stays in simulation
        public static final int MAX_BALLS = 60; // Maximum number of balls to simulate at once

        // Turret position offset (where the ball spawns relative to robot origin)
        public static final double TURRET_HEIGHT_METERS = 0.0; // Height of turret from ground
        public static final double TURRET_FORWARD_OFFSET_METERS = 0.0; // Forward offset from robot center

        // Physics
        public static final double GRAVITY = -9.81; // m/s^2
    }
}
