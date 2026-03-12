package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.SparkBase;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

    // --- CONFIG ---
    private static final int LEADER_CAN_ID = 51;
    private static final int FOLLOWER_CAN_ID = 52;

    // motors
    private final SparkMax leader;
    private final SparkMax follower;


    public Intake() {
        leader = new SparkMax(LEADER_CAN_ID, MotorType.kBrushless);
        follower = new SparkMax(FOLLOWER_CAN_ID, MotorType.kBrushless);

        // configure leader
        leader.setVoltage(0.0);
        var leaderConfig = new SparkMaxConfig();
        leaderConfig.inverted(true);                         // adapt if your motor is reversed
        leaderConfig.idleMode(SparkBaseConfig.IdleMode.kBrake); // brake is generally safer with hard stops
        leaderConfig.smartCurrentLimit(30);
        leaderConfig.voltageCompensation(12);
        leader.configure(leaderConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

        // configure follower to follow the leader
        var followerConfig = new SparkMaxConfig();
        followerConfig.inverted(true).idleMode(SparkBaseConfig.IdleMode.kBrake).smartCurrentLimit(30).voltageCompensation(12);
        // follow by CAN ID:
        followerConfig.follow(LEADER_CAN_ID, true);
        // alternative: if your SDK supports object follow, you can use follower.follow(leader, true);

        follower.configure(followerConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    }

    // --- public control helpers ---

    /** Set voltage directly (volts). No limit-switch checks (mechanical stops are assumed). */
    public void setVoltage(double volts) {
        leader.setVoltage(volts);
    }

    /** Set motor by percent output (-1.0 .. 1.0). This uses 12 V as full scale. */
    public void setPercent(double percent) {
        double volts = percent * 12.0;
        setVoltage(volts);
    }

    /** Convenience: start lowering with a fixed percent (negative or positive depending on motor wiring). */
    public void lower(double percent) {
        setPercent(percent);
    }

    /** Convenience: start raising with a fixed percent. */
    public void raise(double percent) {
        setPercent(percent);
    }

    /** Stops the arm. */
    public void stop() {
        leader.setVoltage(0.0);
    }
}