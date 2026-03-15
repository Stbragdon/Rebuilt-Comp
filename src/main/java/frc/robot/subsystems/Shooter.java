package frc.robot.subsystems;

import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

    public enum ShotPreset {
        CLOSE(5),
        MID(7),
        FAR(9),
        SNOWBLOW(10);

        public final double volts;

        ShotPreset(double volts) {
            this.volts = volts;
        }
    }

    // ===== CAN IDs =====
    private static final int SHOOTER_CAN_ID = 20;
    private static final int KICKER_CAN_ID = 21;

    // ===== Hardware =====
    private final SparkMax shooterMotor = new SparkMax(SHOOTER_CAN_ID, MotorType.kBrushless);
    private final SparkMax kickerMotor  = new SparkMax(KICKER_CAN_ID, MotorType.kBrushless);

    private final RelativeEncoder shooterEncoder = shooterMotor.getEncoder();
    private final SparkClosedLoopController shooterController = shooterMotor.getClosedLoopController();

    // ===== Tuning =====
    // Start here and tune on the robot
    private static final double kP = 0.01;
    private static final double kI = 0.0;
    private static final double kD = 0.0;

    // Use tolerance in RPM for "ready to shoot
    private static final double RPM_TOLERANCE = 500.0;

    // Kicker voltage
    private static final double KICKER_FEED_VOLTS = 8.0;

    // Optional shooter spin-up fallback voltage if you want manual mode later
    private static final double IDLE_HOLD_VOLTS = 0.0;

    private double targetRPM = 0.0;

    public Shooter() {
        configureShooterMotor();
        configureKickerMotor();
    }

    private void configureShooterMotor() {
        SparkMaxConfig config = new SparkMaxConfig();

        config
            .idleMode(com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kCoast)
            .smartCurrentLimit(80)
            .inverted(true);

        EncoderConfig encoderConfig = new EncoderConfig();
        // Native velocity is RPM by default, so this can stay 1.0
        encoderConfig.velocityConversionFactor(1.0);
        encoderConfig.positionConversionFactor(1.0);

    
        config.apply(encoderConfig);

        shooterMotor.configure(
            config,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );
    }

    private void configureKickerMotor() {
        SparkMaxConfig config = new SparkMaxConfig();

        config
            .idleMode(com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kBrake)
            .smartCurrentLimit(60)
            .inverted(true);

        kickerMotor.configure(
            config,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );
    }

    // ===== Shooter controls =====

    public void setShooter(double volts) {
        shooterMotor.setVoltage(volts);
    }

    public void stopShooter() {
        targetRPM = 0.0;
        shooterMotor.setVoltage(IDLE_HOLD_VOLTS);
    }

    public double getShooterRPM() {
        return shooterEncoder.getVelocity();
    }

    public double getTargetRPM() {
        return targetRPM;
    }

    public boolean atTargetSpeed() {
        return Math.abs(getShooterRPM() - targetRPM) <= RPM_TOLERANCE;
    }

    // ===== Kicker controls =====

    public void runKicker(double volts) {
        kickerMotor.setVoltage(volts);
    }
    public void feedBall() {
        kickerMotor.setVoltage(KICKER_FEED_VOLTS);
    }

    public void stopKicker() {
        kickerMotor.stopMotor();
    }

    public void stopAll() {
        stopShooter();
        stopKicker();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter RPM", getShooterRPM());
        SmartDashboard.putNumber("Shooter Target RPM", targetRPM);
        SmartDashboard.putBoolean("Shooter At Speed", atTargetSpeed());
    }
}
