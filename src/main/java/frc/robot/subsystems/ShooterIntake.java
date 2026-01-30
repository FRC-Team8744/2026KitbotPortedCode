package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterIntake extends SubsystemBase {
    private static final int motorID = 14;
    private static final double idleSpeed = -0.1;
    private static final double shootSpeed = -0.8; // 1
    private static final double intakeSpeed = -0.2; // 0.8
    private static final double outtakeSpeed = 0.2; // -0.8
    private static final double maxSpeed = 1.0; // 5000
    private static final double tolerance = 50;

    private final SparkMax motor;
    private double speed;

    //TODO: add motor configurations
    //TODO: add second intake motor
    public ShooterIntake() {
        super("ShooterIntake");
        motor = new SparkMax(motorID, MotorType.kBrushless);

        setDefaultCommand(idle());
    }

    public void run(final double speed) {
        final double validSpeed = MathUtil.clamp(speed, -1, 1);
        this.speed = validSpeed;

        motor.set(validSpeed);
    }

    public boolean atSpeed() {
        return Math.abs(motor.getEncoder().getVelocity() - (speed * maxSpeed)) < tolerance;
    }

    public Command idle() {
        return Commands.run(() -> run(idleSpeed), this);
    }

    public Command shoot() {
        return Commands.run(() -> run(shootSpeed), this);
    }

    //TODO: never seems to continue
    public Command waitForSpeed() {
        return Commands.waitUntil(this::atSpeed);
    }

    public Command intake() {
        return Commands.run(() -> run(intakeSpeed), this);
    }

    public Command outtake() {
        return Commands.run(() -> run(outtakeSpeed), this);
    }

    //TODO: this causes the code to crash
    public Command stop() {
        return Commands.runOnce(() -> this.getCurrentCommand().cancel(), this);
    }
}
