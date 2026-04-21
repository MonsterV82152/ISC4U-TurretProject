package frc.robot.turret;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants;
import frc.robot.Constants.TurretConstants;
import frc.robot.util.TurretParameter;
import frc.robot.util.VelocityPIDController;
import frc.robot.util.BallSim;

public class Turret {
    private static Turret turretInst;
    private TurretInputsAutoLogged inputs;
    private TurretIO io;

    private Turret() {
        inputs = new TurretInputsAutoLogged();
        io = Constants.IS_SIM ? new TurretIOSimulated() : new TurretIOSparkMax();
    }

    public static Turret getInstance() {
        if (turretInst == null) {
            turretInst = new Turret();
        }
        return turretInst;
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs(Constants.IS_SIM ? "Turret_SIM" : "Turret_REAL", inputs);
        io.periodic();
    }

    public void setGains() {
        io.setGains();
    }

    public void setTurretParameters(TurretParameter params) {
        io.setSwivelPosition(params.swivelRad);
        io.setHoodPosition(params.hoodRad);
        io.setFlywheelRPS(params.shotRPS);
    }

    public void setSwivelPosition(Rotation2d position) {
        io.setSwivelPosition(position);
    }

    public void setHoodPosition(Rotation2d position) {
        io.setHoodPosition(position);
    }

    public void setFlywheelRPS(double rps) {
        io.setFlywheelRPS(rps);
    }

    public void shootBall() {
        io.shootBall();
    }

    public boolean isAtSwivelPosition() {
        return io.isAtSwivelPosition();
    }

    public boolean isAtHoodPosition() {
        return io.isAtHoodPosition();
    }

    public boolean isAtFlywheelRPS() {
        return io.isAtFlywheelRPS();
    }

    public boolean isReadyToShoot() {
        return isAtSwivelPosition() && isAtHoodPosition() && isAtFlywheelRPS();
    }

}

interface TurretIO {
    @AutoLog
    public class TurretInputs {
        public boolean swivelConnection = false;
        public double swivelVolts = 0.0;
        public double swivelCurrent = 0.0;
        public double swivelTemp = 0.0;
        public double swivelVelocity = 0.0;
        public double swivelPosition = 0.0;
        public double swivelExt1EncoderPosition = 0.0;
        public double swivelExt2EncoderPosition = 0.0;
        public double swivelError = 0.0;

        public double hoodPosition = 0.0;

        public boolean flywheelConnection = false;
        public double flywheelVolts = 0.0;
        public double flywheelCurrent = 0.0;
        public double flywheelTemp = 0.0;
        public double flywheelVelocity = 0.0;
        public double flywheelError = 0.0;
    }

    public default void updateInputs(TurretInputs inputs) {
    }

    public default void setGains() {
    }

    public default void setSwivelPosition(Rotation2d position) {
    }

    public default void setHoodPosition(Rotation2d position) {
    }

    public default void setFlywheelRPS(double rps) {
    }

    public default void shootBall() {
    }

    public default boolean isAtSwivelPosition() {
        return false;
    }

    public default boolean isAtHoodPosition() {
        return false;
    }

    public default boolean isAtFlywheelRPS() {
        return false;
    }

    public default void periodic() {
    }
}

class TurretIOSparkMax implements TurretIO {
    // private TalonSRX swivelMotor;
    // private TalonSRXConfiguration swivelConfig;
    // private TalonSRX hoodMotor;
    // private TalonSRXConfiguration hoodConfig;
    // private SparkMax flywheelMotor;
    // private SparkMaxConfig flywheelConfig;

    // public TurretIOSparkMax() {
    // swivelMotor = new TalonSRX(Constants.TurretConstants.SWIVEL_MOTOR_ID);
    // swivelConfig = new TalonSRXConfiguration();

    // swivelConfig.slot0.kP = TurretConstants.SWIVEL_KP;
    // swivelConfig.slot0.kI = TurretConstants.SWIVEL_KI;
    // swivelConfig.slot0.kD = TurretConstants.SWIVEL_KD;

    // swivelConfig.primaryPID.selectedFeedbackSensor =
    // FeedbackDevice.CTRE_MagEncoder_Relative;

    // swivelMotor.configAllSettings(swivelConfig);

    // hoodMotor = new TalonSRX(Constants.TurretConstants.HOOD_MOTOR_ID);
    // hoodConfig = new TalonSRXConfiguration();
    // flywheelMotor = new SparkMax(Constants.TurretConstants.FLYWHEEL_MOTOR_ID,
    // SparkMax.MotorType.kBrushless);
    // flywheelConfig = new SparkMaxConfig();
    // }

}

class TurretIOSimulated implements TurretIO {
    private FlywheelSim flywheelSim;
    private DCMotorSim swivelSim;

    private DCMotor flywheelMotorPlant;
    private DCMotor swivelMotorPlant;

    private LoggedNetworkNumber flywheelkP;
    private LoggedNetworkNumber flywheelkI;
    private LoggedNetworkNumber flywheelkD;
    private LoggedNetworkNumber flywheelkS;
    private LoggedNetworkNumber flywheelkV;

    private LoggedNetworkNumber swivelkP;
    private LoggedNetworkNumber swivelkI;
    private LoggedNetworkNumber swivelkD;

    private VelocityPIDController flywheelPID;
    private PIDController swivelPID;

    private Rotation2d hoodPosition;

    private BallSim ballSim;

    public TurretIOSimulated() {
        ballSim = BallSim.getInstance();
        flywheelMotorPlant = DCMotor.getKrakenX60Foc(2);
        swivelMotorPlant = DCMotor.getNeo550(1);
        hoodPosition = Rotation2d.kZero;
        flywheelSim = new FlywheelSim(
                LinearSystemId.createFlywheelSystem(flywheelMotorPlant, TurretConstants.FLYWHEEL_MOMENT_OF_INERTIA, 1),
                flywheelMotorPlant, 0.01);
        Logger.recordOutput("FLywheel MOI", TurretConstants.FLYWHEEL_MOMENT_OF_INERTIA);
        ballSim.setFlywheelSim(flywheelSim);
        
        swivelSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(swivelMotorPlant, TurretConstants.SWIVEL_MOMENT_OF_INERTIA, 100),
                swivelMotorPlant, 0.0, 0.0);
        flywheelPID = new VelocityPIDController(TurretConstants.SIM_FLYWHEEL_KP, TurretConstants.SIM_FLYWHEEL_KI,
                TurretConstants.SIM_FLYWHEEL_KD, TurretConstants.SIM_FLYWHEEL_KS, TurretConstants.SIM_FLYWHEEL_KV);
        swivelPID = new PIDController(TurretConstants.SIM_SWIVEL_KP, TurretConstants.SIM_SWIVEL_KI,
                TurretConstants.SIM_SWIVEL_KD);

        flywheelkP = new LoggedNetworkNumber("TurretSim/flywheelKP", TurretConstants.SIM_FLYWHEEL_KP);
        flywheelkI = new LoggedNetworkNumber("TurretSim/flywheelKI", TurretConstants.SIM_FLYWHEEL_KI);
        flywheelkD = new LoggedNetworkNumber("TurretSim/flywheelKD", TurretConstants.SIM_FLYWHEEL_KD);
        flywheelkS = new LoggedNetworkNumber("TurretSim/flywheelKS", TurretConstants.SIM_FLYWHEEL_KS);
        flywheelkV = new LoggedNetworkNumber("TurretSim/flywheelKV", TurretConstants.SIM_FLYWHEEL_KV);

        swivelkP = new LoggedNetworkNumber("TurretSim/swivelKP", TurretConstants.SIM_SWIVEL_KP);
        swivelkI = new LoggedNetworkNumber("TurretSim/swivelKI", TurretConstants.SIM_SWIVEL_KI);
        swivelkD = new LoggedNetworkNumber("TurretSim/swivelKD", TurretConstants.SIM_SWIVEL_KD);
    }

    @Override
    public void setGains() {
        flywheelPID.setGains(flywheelkP.get(), flywheelkI.get(), flywheelkD.get(), flywheelkS.get(), flywheelkV.get());
        swivelPID.setP(swivelkP.get());
        swivelPID.setI(swivelkI.get());
        swivelPID.setD(swivelkD.get());
    }

    @Override
    public void setSwivelPosition(Rotation2d position) {
        swivelPID.setSetpoint(position.getRadians());
    }

    @Override
    public void setFlywheelRPS(double rps) {
        flywheelPID.setSetpoint(rps);
    }

    @Override
    public void setHoodPosition(Rotation2d position) {
        this.hoodPosition = position;
    }

    @Override
    public void shootBall() {
        ballSim.shootBall(hoodPosition, Rotation2d.fromRadians(swivelSim.getAngularPositionRad()),
                flywheelSim.getAngularVelocityRPM() / 60.0);
    }

    @Override
    public boolean isAtSwivelPosition() {
        return Math.abs(swivelPID.getSetpoint()
                - swivelSim.getAngularPositionRotations()) < Constants.TurretConstants.SWIVEL_TOLERANCE.getRadians();
    }

    @Override
    public boolean isAtFlywheelRPS() {
        return Math.abs(flywheelPID.getSetpoint()
                - flywheelSim.getAngularVelocityRPM() / 60.0) < Constants.TurretConstants.FLYWHEEL_RPS_TOLERANCE;
    }

    @Override
    public boolean isAtHoodPosition() {
        return true;
    }

    @Override
    public void periodic() {
        double swivelOutput = swivelPID.calculate(swivelSim.getAngularPositionRad());
        swivelSim.setInput(swivelOutput);
        double flywheelOutput = flywheelPID.calculate(flywheelSim.getAngularVelocityRPM() / 60.0);
        flywheelSim.setInput(flywheelOutput);

        flywheelSim.update(Constants.LOOP_PERIOD);
        swivelSim.update(Constants.LOOP_PERIOD);
        ballSim.periodic();
    }

    @Override
    public void updateInputs(TurretInputs inputs) {
        inputs.swivelPosition = swivelSim.getAngularPositionRad();
        inputs.swivelVelocity = swivelSim.getAngularVelocityRPM() / 60.0;
        inputs.hoodPosition = hoodPosition.getRadians();
        inputs.flywheelVelocity = flywheelSim.getAngularVelocityRPM() / 60.0;
    }
}