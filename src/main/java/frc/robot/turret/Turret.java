package frc.robot.turret;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.CANBus;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants;
import frc.robot.Constants.ShooterUtilConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.util.TurretParameter;
import frc.robot.util.VelocityPIDController;
import frc.robot.util.BallSim;
import frc.robot.util.ShotCalculator;

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
  private final CANBus canBus;

  /*
   * ------------------------ SWIVEL (TalonSRX - Phoenix 5)
   * ------------------------
   */
  // Two mag encoders with different gear ratios for absolute position tracking
  // (Vernier method)
  // Encoder 1: 11 teeth gear, Encoder 2: 17 teeth gear, Turret: 120 teeth
  private TalonSRX swivelMotor; // Main swivel motor with 17-tooth encoder
  private PIDController swivelPID;
  private TalonSRX swivelEncoder; // TalonSRX reading 11-tooth encoder
  private double swivelPosition;
  private double angleOffset;

  // Encoder filters for noise reduction
  // MedianFilter removes spikes/outliers, LinearFilter (moving average) smooths
  // the signal

  private MedianFilter enc1MedianFilter;
  private MedianFilter enc2MedianFilter;
  private LinearFilter enc1MovingAvgFilter;
  private LinearFilter enc2MovingAvgFilter;

  /* ------------------------ HOOD ------------------------ */
  // TalonSRX uses Phoenix 5 API (com.ctre.phoenix)
  private TalonSRX hoodMotor;

  /* ------------------------ SHOOT ------------------------ */
  private SparkMax shootMotor;
  private SimpleMotorFeedforward shootFF;
  private PIDController shootPID;

  public TurretIOSparkMax() {
    canBus = new CANBus("canbus");

    /*
     * ------------------------ SWIVEL (TalonSRX - Phoenix 5)
     * ------------------------
     */
    // Initialize encoder filters
    enc1MedianFilter = new MedianFilter(ShooterUtilConstants.MEDIAN_FILTER_SIZE);
    enc2MedianFilter = new MedianFilter(ShooterUtilConstants.MEDIAN_FILTER_SIZE);
    enc1MovingAvgFilter = LinearFilter.movingAverage(ShooterUtilConstants.MOVING_AVG_TAPS);
    enc2MovingAvgFilter = LinearFilter.movingAverage(ShooterUtilConstants.MOVING_AVG_TAPS);

    // Encoder 1: 11-tooth gear (for Vernier absolute position)
    swivelEncoder = new TalonSRX(TurretConstants.SWIVEL_ENCODER_ID);
    swivelEncoder.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 30);

    // Swivel motor with Encoder 2: 17-tooth gear
    swivelMotor = new TalonSRX(TurretConstants.SWIVEL_MOTOR_ID);
    swivelMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 30);
    swivelPID = new PIDController(TurretConstants.SWIVEL_P, TurretConstants.SWIVEL_I, TurretConstants.SWIVEL_D);

    // Configure current limit
    swivelMotor.configContinuousCurrentLimit(40, 30);
    swivelMotor.configPeakCurrentLimit(60, 30);
    swivelMotor.configPeakCurrentDuration(100, 30);
    swivelMotor.enableCurrentLimit(true);

    // Set neutral mode to brake
    swivelMotor.setNeutralMode(NeutralMode.Brake);

    // Prime the filters with initial readings
    double initialEnc1 = swivelTicksToDegreesAbsolute(swivelMotor.getSelectedSensorPosition());
    double initialEnc2 = swivelTicksToDegreesAbsolute(swivelEncoder.getSelectedSensorPosition());
    for (int i = 0; i < ShooterUtilConstants.MEDIAN_FILTER_SIZE; i++) {
      enc1MedianFilter.calculate(initialEnc1);
      enc2MedianFilter.calculate(initialEnc2);
    }
    for (int i = 0; i < ShooterUtilConstants.MOVING_AVG_TAPS; i++) {
      enc1MovingAvgFilter.calculate(initialEnc1);
      enc2MovingAvgFilter.calculate(initialEnc2);
    }

    /*
     * ------------------------ HOOD (TalonSRX - Phoenix 5) ------------------------
     */
    hoodMotor = new TalonSRX(TurretConstants.HOOD_MOTOR_ID);

    // Configure feedback sensor - use absolute to retain position after power cycle
    hoodMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 30);

    // Configure current limit
    hoodMotor.configContinuousCurrentLimit((int) TurretConstants.HOOD_CURRENT_LIMIT, 30);
    hoodMotor.configPeakCurrentLimit((int) (TurretConstants.HOOD_CURRENT_LIMIT * 1.5), 30);
    hoodMotor.configPeakCurrentDuration(100, 30);
    hoodMotor.enableCurrentLimit(true);

    // Configure PID (Slot 0)
    hoodMotor.config_kP(0, TurretConstants.HOOD_P, 30);
    hoodMotor.config_kI(0, TurretConstants.HOOD_I, 30);
    hoodMotor.config_kD(0, TurretConstants.HOOD_D, 30);
    hoodMotor.config_kF(0, 0.0, 30);

    // Set neutral mode to brake
    hoodMotor.setNeutralMode(NeutralMode.Brake);

    // Note: Not zeroing sensor position - absolute encoder retains position after
    // power cycle

    /* ------------------------ SHOOT ------------------------ */
    shootMotor = new SparkMax(TurretConstants.SHOOT_MOTOR_ID, MotorType.kBrushless);
    shootPID = new PIDController(TurretConstants.SHOOT_P, TurretConstants.SHOOT_I, TurretConstants.SHOOT_D);
    shootFF = new SimpleMotorFeedforward(0, TurretConstants.SHOOT_V);
  }

  public void setSwivelPosition(double position) {
    double target = determineSafeSwivelTarget(position);

    if (Math.abs(target + 360 - swivelPosition) < Math.abs(target - swivelPosition)
        && target + 360 <= TurretConstants.SWIVEL_MAX_ANGLE) {
      target += 360;
    } else if (Math.abs(target - 360 - swivelPosition) < Math.abs(target - swivelPosition)
        && target - 360 >= TurretConstants.SWIVEL_MIN_ANGLE) {
      target -= 360;
    }
    Logger.recordOutput("SwivelTargetAngle", target);
    swivelPID.setSetpoint(target);
  }

  public void setHoodPosition(double positionDegrees) {
    if (positionDegrees > TurretConstants.HOOD_MAX_ANGLE) positionDegrees = TurretConstants.HOOD_MAX_ANGLE;
    if (positionDegrees < TurretConstants.HOOD_MIN_ANGLE) positionDegrees = TurretConstants.HOOD_MIN_ANGLE;
    double motorPosition = 10+(positionDegrees-TurretConstants.HOOD_MIN_ANGLE)*270/(TurretConstants.HOOD_MAX_ANGLE-TurretConstants.HOOD_MIN_ANGLE);

    // Convert degrees to encoder ticks for TalonSRX position control
    double positionTicks = hoodDegreesToTicks(motorPosition);
    hoodMotor.set(ControlMode.Position, positionTicks);
  }

  public void setShootVelocity(double velocity) {
    shootPID.setSetpoint(velocity);
  }

  public double getSwivelCurrent() {
    return swivelMotor.getSupplyCurrent();
  }

  public double getSwivelVelocity() {
    // Convert from ticks/100ms to degrees/second
    return swivelMotor.getSelectedSensorVelocity() * TurretConstants.SWIVEL_TICKS_TO_DEGREES * 10.0;
  }

  public void resetSwivelPosition() {
    double enc1Raw = swivelEncoder.getSelectedSensorPosition();
    double enc2Raw = swivelMotor.getSelectedSensorPosition();
    double[] filteredEncoders = getFilteredEncoderReadings(enc1Raw, enc2Raw);
    double filteredEnc1 = filteredEncoders[0];
    double filteredEnc2 = filteredEncoders[1];
    swivelPosition = ShotCalculator.calculateTurretAngle(swivelTicksToDegreesAbsolute(filteredEnc1),
        swivelTicksToDegreesAbsolute(filteredEnc2));
    angleOffset = swivelPosition - ShotCalculator.calculateTurretAngleRelative(swivelTicksToDegrees(filteredEnc1));
    Logger.recordOutput("resetSwivelPosition", swivelPosition);
  }

  public void zeroSwivelPosition() {
    swivelEncoder.setSelectedSensorPosition(0);
    swivelMotor.setSelectedSensorPosition(0);
    angleOffset = 0;
  }

  public void zeroHoodPosition() {
    hoodMotor.setSelectedSensorPosition(hoodDegreesToTicks(10));
  }

  public double getHoodVelocity() {
    // Convert from ticks/100ms to degrees/second
    return hoodMotor.getSelectedSensorVelocity() * TurretConstants.HOOD_TICKS_TO_DEGREES * 10.0;
  }

  public double getSwivelPosition() {
    return swivelPosition;
  }

  // Conversion helpers
  private double swivelTicksToDegreesAbsolute(double ticks) {
    if (ticks < 0) {
      ticks = ticks + Math.ceil(ticks / TurretConstants.ENCODER_TICKS_PER_REV) * TurretConstants.ENCODER_TICKS_PER_REV;
    }
    return (ticks % TurretConstants.ENCODER_TICKS_PER_REV) * TurretConstants.SWIVEL_TICKS_TO_DEGREES;
  }

  private double swivelTicksToDegrees(double ticks) {
    return ticks * TurretConstants.SWIVEL_TICKS_TO_DEGREES;
  }

  private double swivelDegreesToTicks(double degrees) {
    return degrees / TurretConstants.SWIVEL_TICKS_TO_DEGREES;
  }

  private double hoodTicksToDegrees(double ticks) {
    return ticks * TurretConstants.HOOD_TICKS_TO_DEGREES;
  }

  private double hoodDegreesToTicks(double degrees) {
    return degrees / TurretConstants.HOOD_TICKS_TO_DEGREES;
  }

  public double determineSafeSwivelTarget(double target) {
    if (target < 0) {
      target += Math.ceil(-target / 360.0) * 360;
    }
    return (target) % 360;
  }

  /**
   * Gets filtered encoder readings using median filter (removes spikes)
   * followed by moving average (smooths signal).
   * 
   * @return array of [filteredEnc1Degrees, filteredEnc2Degrees]
   */
  private double[] getFilteredEncoderReadings(double rawEnc1, double rawEnc2) {

    // Apply median filter first to remove spikes
    double medianEnc1 = enc1MedianFilter.calculate(rawEnc1);
    double medianEnc2 = enc2MedianFilter.calculate(rawEnc2);

    // Apply moving average filter for additional smoothing
    double filteredEnc1 = enc1MovingAvgFilter.calculate(medianEnc1);
    double filteredEnc2 = enc2MovingAvgFilter.calculate(medianEnc2);

    return new double[] { Math.round(filteredEnc1 * 10) / 10, Math.round(filteredEnc2 * 10) / 10 };
  }

  public void setPIDController() {
    shootPID.setPID(shootPID.getP(), shootPID.getI(), shootPID.getD());
  }

  public void periodic() {

    double enc1Raw = swivelEncoder.getSelectedSensorPosition();
    double enc2Raw = swivelMotor.getSelectedSensorPosition();

    // Log raw and filtered values for debugging
    Logger.recordOutput("ENC1_Raw", enc1Raw);
    Logger.recordOutput("ENC2_Raw", enc2Raw);

    // Get filtered encoder readings
    double[] filteredEncoders = getFilteredEncoderReadings(enc1Raw, enc2Raw);
    double filteredEnc1 = filteredEncoders[0];
    double filteredEnc2 = filteredEncoders[1];

    Logger.recordOutput("ENC1", filteredEnc1);
    Logger.recordOutput("ENC2", filteredEnc2);
    Logger.recordOutput("AngleOffset", angleOffset);

    swivelPosition = ShotCalculator.calculateTurretAngleRelative(swivelTicksToDegrees(filteredEnc1)) + angleOffset;
    shootMotor.set(shootPID.calculate(shootMotor.getEncoder().getVelocity()) + shootFF.calculate(shootPID.getSetpoint()));
    double swivelOutput = swivelPID.calculate(swivelPosition);
    swivelMotor.set(ControlMode.PercentOutput, swivelOutput / 12.0); // Normalize to -1 to 1
    Logger.recordOutput("HoodVoltage", hoodMotor.getMotorOutputVoltage());
    Logger.recordOutput("HoodControl", hoodMotor.getControlMode());

  }

  public void initialize() {
    ErrorCode errorCode = hoodMotor.setSelectedSensorPosition(hoodDegreesToTicks(10));
    Logger.recordOutput("hoodZeroErrorCode", errorCode);
    resetSwivelPosition();
    hoodMotor.setSensorPhase(true);
    hoodMotor.setInverted(false);
    hoodMotor.setSelectedSensorPosition(hoodDegreesToTicks(10));
  }

  public void updateInputs(TurretInputs inputs) {
    inputs.swivelPosition = swivelPosition;
    inputs.swivelConnection = swivelMotor.getLastError() == ErrorCode.OK;
    inputs.swivelVolts = swivelMotor.getMotorOutputVoltage();
    inputs.swivelCurrent = swivelMotor.getSupplyCurrent();
    // Convert velocity from ticks/100ms to degrees/second
    inputs.swivelVelocity = swivelMotor.getSelectedSensorVelocity() * TurretConstants.SWIVEL_TICKS_TO_DEGREES * 10.0;
    // Convert position from ticks to degrees
    inputs.hoodPosition = hoodTicksToDegrees(hoodMotor.getSelectedSensorPosition());

    // SparkMax connection - assume connected if we can read velocity
    inputs.flywheelConnection = true;
    inputs.flywheelVelocity = shootMotor.getEncoder().getVelocity();
  }

  public boolean isAtHoodPosition() {
    return true;
  }

  public boolean isAtShootVelocity() {
    return true;
  }
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