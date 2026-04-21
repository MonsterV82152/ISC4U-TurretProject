package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
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
        public static final int SWIVEL_MOTOR_ID = 1;
        public static final int SWIVEL_EXT_ENCODER_1_ID = 1;
        public static final int SWIVEL_EXT_ENCODER_2_ID = 2;

        public static final double SWIVEL_KP = 0.1;
        public static final double SWIVEL_KI = 0.0;
        public static final double SWIVEL_KD = 0.0;

        public static final double SIM_SWIVEL_KP = 50.0;
        public static final double SIM_SWIVEL_KI = 0.0;
        public static final double SIM_SWIVEL_KD = 0.0;

        public static final Rotation2d SWIVEL_TOLERANCE = Rotation2d.fromDegrees(5);

        public static final int FLYWHEEL_MOTOR_ID = 3;

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

        public static final int HOOD_MOTOR_ID = 2;

        public static final double METAL_PLATES_FLYWHEEL_WEIGHT_KG = Units.lbsToKilograms(0.2);
        public static final double FLYWHEEL_FLYWHEEL_WEIGHT_KG = Units.lbsToKilograms(0.2);
        public static final double FLYWHEEL_RADIUS_METERS = Units.inchesToMeters(1.5); // 1.5 inches in meters
        public static final double METAL_PLATE_RADIUS_METERS = FLYWHEEL_RADIUS_METERS - 0.01;
        // public static final double FLYWHEEL_MOMENT_OF_INERTIA = FLYWHEEL_FLYWHEEL_WEIGHT_KG * FLYWHEEL_RADIUS_METERS * FLYWHEEL_RADIUS_METERS / 2.0 + 2 * METAL_PLATES_FLYWHEEL_WEIGHT_KG * METAL_PLATE_RADIUS_METERS * METAL_PLATE_RADIUS_METERS;
        public static final double FLYWHEEL_MOMENT_OF_INERTIA = 0.0001656946 * 2;
        public static final double SWIVEL_MOMENT_OF_INERTIA = 0.00395063532;
 
    }

    public final class BallSimConstants {
        // Ball physical properties
        public static final double BALL_MASS_KG = Units.lbsToKilograms(0.5); // Mass of a typical game ball (e.g., cargo ball)
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
