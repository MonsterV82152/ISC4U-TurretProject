package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.event.EventLoop;
import frc.robot.turret.Turret;
import frc.robot.util.ShotCalculator;
import frc.robot.util.States;

public class RobotContainer {

    // Subsystems
    private final Turret turret;

    private LoggedNetworkNumber targetX;
    private LoggedNetworkNumber targetY;
    private LoggedNetworkNumber targetZ;

    // Controller
    private final XboxController controller = new XboxController(0);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        turret = Turret.getInstance();
        targetX = new LoggedNetworkNumber("TargetX", 2);
        targetY = new LoggedNetworkNumber("TargetY", 1);
        targetZ = new LoggedNetworkNumber("TargetZ", 4);
    }

    public void bindControls() {

    }

    public void teleopPeriodic() {
        // if (controller.getAButtonPressed()) {
        //     turret.shootBall();
        // }
        // if (controller.getBButton()) {
        //     double x = targetX.get();
        //     double y = targetY.get();
        //     double z = targetZ.get();
        //     ShotCalculator.setTarget(new Translation3d(x, y, z));
        // }
        // if (controller.getBButtonPressed()) {
        // States.RobotState.turretState = States.RobotState.TurretState.MANUAL;
        // }
        // if (controller.getYButtonPressed()) {
        // States.RobotState.turretState = States.RobotState.TurretState.AIMING;
        // }

        if (controller.getAButtonPressed()) {
        turret.setGains();
        turret.setFlywheelRPS(80);
        }
        if (controller.getBButtonPressed()) {
        turret.setGains();
        turret.setFlywheelRPS(10);
        }
        if (controller.getAButtonReleased() || controller.getBButtonReleased()) {
        turret.setFlywheelRPS(0);
        }

        // if (controller.getAButtonPressed()) {
        // turret.setGains();
        // turret.setSwivelPosition(Rotation2d.fromDegrees(0));
        // }
        // if (controller.getBButtonPressed()) {
        // turret.setGains();
        // turret.setSwivelPosition(Rotation2d.fromDegrees(90));
        // }
        // if (controller.getXButtonPressed()) {
        // turret.setGains();
        // turret.setSwivelPosition(Rotation2d.fromDegrees(180));
        // }
        // if (controller.getYButtonPressed()) {
        // turret.setGains();
        // turret.setSwivelPosition(Rotation2d.fromDegrees(270));
        // }
    }

    public void autonomousInit() {
    }

    public void teleopInit() {
    }

    public void periodic() {
        turret.periodic();
        if (States.RobotState.turretState == States.RobotState.TurretState.AIMING) {
            turret.setTurretParameters(ShotCalculator.calculateShot());
            turret.setGains();
        }
    }
}