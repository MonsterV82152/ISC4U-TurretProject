package frc.robot;

import com.ctre.phoenix.motorcontrol.can.SlotConfiguration;

import edu.wpi.first.math.geometry.Rotation2d;

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

        public static final double SIM_SWIVEL_KP = 0.1;
        public static final double SIM_SWIVEL_KI = 0.0;
        public static final double SIM_SWIVEL_KD = 0.0;

        public static final Rotation2d SWIVEL_TOLERANCE = Rotation2d.fromDegrees(5);

        public static final int FLYWHEEL_MOTOR_ID = 3;

        public static final double FLYWHEEL_KP = 0.1;
        public static final double FLYWHEEL_KI = 0.0;
        public static final double FLYWHEEL_KD = 0.0;
        public static final double FLYWHEEL_KS = 0.0;
        public static final double FLYWHEEL_KV = 0.0;

        public static final double SIM_FLYWHEEL_KP = 0.1;
        public static final double SIM_FLYWHEEL_KI = 0.0;
        public static final double SIM_FLYWHEEL_KD = 0.0;
        public static final double SIM_FLYWHEEL_KS = 0.0;
        public static final double SIM_FLYWHEEL_KV = 0.0;

        public static final double FLYWHEEL_RPS_TOLERANCE = 0.1;

        public static final int HOOD_MOTOR_ID = 2;

        public static final double FLYWHEEL_MOMENT_OF_INERTIA = 7.3161144E-5;
        public static final double SWIVEL_MOMENT_OF_INERTIA = 0.00395063532;
    }
}
