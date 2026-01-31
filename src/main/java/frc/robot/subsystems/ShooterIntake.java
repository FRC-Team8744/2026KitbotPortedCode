package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import static edu.wpi.first.units.Units.Seconds;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.ImmutableTime;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterIntake extends SubsystemBase {
    private static final int shooterID = 14;
    private static final int intakeID = 19;
    private static final double idleSpeed = -0;
    private static final double shootSpeed = -0.8; // 1
    private static final double intakeSpeed = -0.8; // 0.8
    private static final double outtakeSpeed = 0.2; // -0.8
    private static final double maxSpeed = 1.0; // 5000
    private static final double tolerance = 50;

    private final SparkMax Shooter;
    private final SparkMax Intake;
    private double speed;

    //TODO: add motor configurations
    //TODO: add second intake motor
    public ShooterIntake() {
        super("ShooterIntake");
        Shooter = new SparkMax(shooterID, MotorType.kBrushless);
        Intake = new SparkMax(intakeID, MotorType.kBrushless);
        setDefaultCommand(standby());
    }
    
    public void run(final double speed) {
        final double validSpeed = MathUtil.clamp(speed, -1, 1);
        this.speed = validSpeed;
        Intake.set(-validSpeed);
        Shooter.set(validSpeed);
    }

    public boolean atSpeed() {
        return Math.abs(Shooter.getEncoder().getVelocity() - (speed * maxSpeed)) < tolerance;
    }

    public Command standby() {
        return Commands.run(() -> run(idleSpeed), this);
    }

    public Command shoot() {
        return Commands.run(() -> run(shootSpeed), this);
    }

    //TODO: never seems to continue
    public Command waitForSpeed() {
      //  return Commands.waitUntil(this::atSpeed);
       return Commands.waitTime(new ImmutableTime(1, 1, Seconds));
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
