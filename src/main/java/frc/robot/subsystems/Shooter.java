package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

    public enum ShotPreset {
        CLOSE(5.0, 4.0, 6.5),
        MID(7.0, 6.5, 9.5),
        FAR(9.0, 9.5, 13.0),
        SNOWBLOW(10.0, 13.0, 18.0);

        public final double volts;
        public final double minFeet;
        public final double maxFeet;

        ShotPreset(double volts, double minFeet, double maxFeet) {
            this.volts = volts;
            this.minFeet = minFeet;
            this.maxFeet = maxFeet;
        }
    }

    // ===== CAN IDs =====
    private static final int SHOOTER_CAN_ID = 20;
    private static final int KICKER_CAN_ID = 21;

    // ===== Vision =====
    private final PhotonCamera camera = new PhotonCamera("Target");

    // ✅ MULTI-TAG SET (same as drive)
    private static final int[] HUB_TAG_IDS = {8, 10, 11, 24, 26, 27};

    // ===== Hardware =====
    private final SparkFlex shooterMotor = new SparkFlex(SHOOTER_CAN_ID, MotorType.kBrushless);
    private final SparkFlex kickerMotor = new SparkFlex(KICKER_CAN_ID, MotorType.kBrushless);

    // ===== Constants =====
    private static final double KICKER_FEED_VOLTS = 8.0;
    private static final double IDLE_HOLD_VOLTS = 0.0;

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

    public void setShooterPreset(ShotPreset preset) {
        shooterMotor.setVoltage(preset.volts);
    }

    public void stopShooter() {
        shooterMotor.setVoltage(IDLE_HOLD_VOLTS);
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

    // ===== Vision (BEST TAG SELECTION) =====

    public double getDistanceFromBestTag(int[] validTagIds) {
        var result = camera.getLatestResult();

        if (!result.hasTargets()) {
            return -1.0;
        }

        PhotonTrackedTarget bestTarget = null;
        double bestScore = Double.MAX_VALUE;

        for (PhotonTrackedTarget target : result.getTargets()) {

            int id = target.getFiducialId();

            for (int validId : validTagIds) {
                if (id == validId) {

                    double score = Math.abs(target.getYaw());

                    if (score < bestScore) {
                        bestScore = score;
                        bestTarget = target;
                    }

                    break;
                }
            }
        }

        if (bestTarget != null) {
            double distanceMeters =
                bestTarget.getBestCameraToTarget().getTranslation().getNorm();

            return Units.metersToFeet(distanceMeters);
        }

        return -1.0;
    }

    public double getHubDistanceFeet() {
        return getDistanceFromBestTag(HUB_TAG_IDS);
    }

    public boolean hasHubTag() {
        return getHubDistanceFeet() > 0;
    }

    // ===== Shot Logic =====

    public boolean inRange(double distanceFeet, ShotPreset preset) {
        if (distanceFeet < 0) {
            return false;
        }

        return distanceFeet >= preset.minFeet && distanceFeet < preset.maxFeet;
    }

    public Optional<ShotPreset> getPresetForDistance(double distanceFeet) {
        if (distanceFeet < 0) {
            return Optional.empty();
        }

        for (ShotPreset preset : ShotPreset.values()) {
            if (inRange(distanceFeet, preset)) {
                return Optional.of(preset);
            }
        }

        return Optional.empty();
    }

    public boolean isInAnyShotRange(double distanceFeet) {
        return getPresetForDistance(distanceFeet).isPresent();
    }

    public void applyAutoShotFromDistance() {
        double distanceFeet = getHubDistanceFeet();
        double voltage = getVoltageForDistance(distanceFeet);

        if (distanceFeet > 0) {
            setShooter(voltage);
        } else {
            stopShooter();
        }
    }

    public double getVoltageForDistance(double distanceFeet) {
        if (distanceFeet < 0) {
            return 0.0;
        }

        double voltage = 0.233 * distanceFeet + 4.55;

        return Math.max(0.0, Math.min(12.0, voltage));
    }

    @Override
    public void periodic() {
        double distanceFeet = getHubDistanceFeet();
        Optional<ShotPreset> currentPreset = getPresetForDistance(distanceFeet);

        SmartDashboard.putNumber("Distance To Hub Tag (ft)", distanceFeet);
        SmartDashboard.putBoolean("Close Ready", inRange(distanceFeet, ShotPreset.CLOSE));
        SmartDashboard.putBoolean("Mid Ready", inRange(distanceFeet, ShotPreset.MID));
        SmartDashboard.putBoolean("Far Ready", inRange(distanceFeet, ShotPreset.FAR));

        if (currentPreset.isPresent()) {
            SmartDashboard.putString("Active Shot Preset", currentPreset.get().name());
            SmartDashboard.putNumber("Suggested Shooter Volts", currentPreset.get().volts);
        } else {
            SmartDashboard.putString("Active Shot Preset", "NONE");
            SmartDashboard.putNumber("Suggested Shooter Volts", 0.0);
        }
    }
}