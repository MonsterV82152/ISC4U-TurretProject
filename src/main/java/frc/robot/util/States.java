package frc.robot.util;

public class States {
    public class RobotState {
        public enum TurretState {
            IDLE, MANUAL, AIMING
        }
        public static TurretState turretState = TurretState.MANUAL;
    }
}
