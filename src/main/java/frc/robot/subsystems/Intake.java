package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

    // --- CHANGE THESE TO YOUR REAL CAN IDS ---
    private static final int ARM_CAN_ID = 31;      // raises / lowers intake
    private static final int ROLLER_CAN_ID = 30;   // spins intake roller

    // motors
    private final SparkMax armMotor;
    private final SparkMax rollerMotor;

    public Intake() {
        armMotor = new SparkMax(ARM_CAN_ID, MotorType.kBrushless);
        rollerMotor = new SparkMax(ROLLER_CAN_ID, MotorType.kBrushless);

        // --- Arm motor config ---
        SparkMaxConfig armConfig = new SparkMaxConfig();
        armConfig.inverted(true); // flip if arm moves wrong direction
        armConfig.idleMode(SparkBaseConfig.IdleMode.kBrake);
        armConfig.smartCurrentLimit(30);
        armConfig.voltageCompensation(12);

        armMotor.configure(
            armConfig,
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters
        );

        // --- Roller motor config ---
        SparkMaxConfig rollerConfig = new SparkMaxConfig();
        rollerConfig.inverted(false); // flip if intake spins wrong way
        rollerConfig.idleMode(SparkBaseConfig.IdleMode.kCoast);
        rollerConfig.smartCurrentLimit(30);
        rollerConfig.voltageCompensation(12);

        rollerMotor.configure(
            rollerConfig,
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters
        );

        stopArm();
        stopRoller();
    }

    // ---------------- ARM METHODS ----------------

    /** Set arm motor voltage directly */
    public void setArmVoltage(double volts) {
        armMotor.setVoltage(volts);
    }

    /** Set arm motor percent output (-1.0 to 1.0) */
    public void setArmPercent(double percent) {
        setArmVoltage(percent * 12.0);
    }

    public void raise(double volts) {
        setArmVoltage(volts);
    }

    public void lower(double volts) {
        setArmVoltage(volts);
    }

    public void stopArm() {
        armMotor.setVoltage(0.0);
    }

    // ---------------- ROLLER METHODS ----------------

    /** Set roller motor voltage directly */
    public void setRollerVoltage(double volts) {
        rollerMotor.setVoltage(volts);
    }

    /** Set roller motor percent output (-1.0 to 1.0) */
    public void setRollerPercent(double percent) {
        setRollerVoltage(percent * 12.0);
    }

    /** Pull game pieces in */
    public void intake(double volts) {
        setRollerVoltage(-volts);
    }

    /** Push game pieces out */
    public void outtake(double volts) {
        setRollerVoltage(volts);
    }

    public void stopRoller() {
        rollerMotor.setVoltage(0.0);
    }

    // ---------------- STOP EVERYTHING ----------------

    public void stop() {
        stopArm();
        stopRoller();
    }

    // ---------- COMMANDS ----------
    public Command raiseCommand(double volts) {
        return runEnd(
            () -> raise(volts),
            this::stopArm
        );
    }

    public Command lowerCommand(double volts) {
        return runEnd(
            () -> lower(-Math.abs(volts)),
            this::stopArm
        );
    }

    public Command intakeCommand(double volts) {
        return runEnd(
            () -> intake(volts),
            this::stopRoller
        );
    }

    public Command outtakeCommand(double volts) {
        return runEnd(
            () -> outtake(volts),
            this::stopRoller
        );
    }

    public Command stopCommand() {
        return runOnce(this::stop);
    }
}