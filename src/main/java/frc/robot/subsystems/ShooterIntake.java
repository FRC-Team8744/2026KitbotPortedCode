package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import static edu.wpi.first.units.Units.Seconds;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.ImmutableTime;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterIntake extends SubsystemBase {
    private static final int shooterID = 14;
    private static final int intakeID = 19;
    private static final double idleSpeed = -0;
    private static final double shootSpeed = -0.8; // 1
    private static final double intakeSpeed = -0.5; // 0.8
    private static final double outtakeSpeed = 0.2; // -0.8
    private static final double maxSpeed = 1.0; // 5000
    private static final double tolerance = 50;

    private final SparkMax Shooter;
    private final SparkMax Intake;
    private double speed;

    //TODO: add motor configurations
    public ShooterIntake() {
        super("ShooterIntake");
        Shooter = new SparkMax(shooterID, MotorType.kBrushless);
        Intake = new SparkMax(intakeID, MotorType.kBrushless);
        setDefaultCommand(standby());
    }
    @Override
    public void periodic(){
        SmartDashboard.putBoolean("atSpeed", atSpeed());
        SmartDashboard.putNumber("Shootervel", Shooter.getEncoder().getVelocity());
        SmartDashboard.putNumber("Intakevel",Intake.getEncoder().getVelocity());
    }
    



    //TODO: need to have intake on it's own run method
    public void runShooter(final double speed) {
        final double validSpeed = MathUtil.clamp(speed, -1, 1);
        this.speed = validSpeed;
        Shooter.set(validSpeed);
    }

    public void runIntake(final double speed) {
        final double validSpeed = MathUtil.clamp(speed, -1, 1);
        this.speed = validSpeed;
        Intake.set(-validSpeed);
        Shooter.set(validSpeed);
    }

    public void stopIntake() {
        Intake.stopMotor();
    }

    public boolean atSpeed() {
        return Math.abs(Shooter.getEncoder().getVelocity() - (speed * maxSpeed)) < tolerance;
    }

    public Command standby() {
        return Commands.run(() -> runShooter(idleSpeed), this);
    }

    public Command shoot() {
        return Commands.run(() -> runShooter(shootSpeed), this);
    }

    //TODO: never seems to continue look atr Commands.waitSeconds 
    public Command waitForSpeed() {
       return Commands.waitUntil(this::atSpeed);// 
      // return Commands.waitSeconds(3.67);
    }

    public Command intake() {
        return Commands.run(() -> runIntake(intakeSpeed), this);
    }

    public Command outtake() {
        return Commands.run(() -> runIntake(outtakeSpeed), this);
    }

    //TODO: this causes the code to crash
    public Command stop() {
        return Commands.runOnce(() -> this.getCurrentCommand().cancel(), this);
    }
}
