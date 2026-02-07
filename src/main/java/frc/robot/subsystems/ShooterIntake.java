package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import static edu.wpi.first.units.Units.Seconds;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.ImmutableTime;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ConstantsOffboard;

public class ShooterIntake extends SubsystemBase {
    private static final int shooterID = 14;
    private static final int intakeID = 19;
    private static final double idleSpeed = -0;
    private static final double shootSpeed = 3000; // 1
    private static final double intakeSpeed = 3000; // 0.8
    private static final double outtakeSpeed = 0.2; // -0.8
    // private static final double maxSpeed = 1.0; // 5000
    private static final double tolerance = 50;

    private static final double kF = 0.0002;
    private static final double kP = 0.00005;

    private final SparkMax Shooter;
    private final SparkMax Intake;
    private double goalspeed;

    //TODO: add motor configurations
    public ShooterIntake() {
        super("ShooterIntake");
        Shooter = new SparkMax(shooterID, MotorType.kBrushless);
        Intake = new SparkMax(intakeID, MotorType.kBrushless);
        configMotor();
        setDefaultCommand(standby());
    }
    
    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putBoolean("atSpeed", atSpeed());
        SmartDashboard.putNumber("ShooterVel", Shooter.getEncoder().getVelocity());
        SmartDashboard.putNumber("Intake.Vel", Intake.getEncoder().getVelocity());
    }

    //TODO: need to have intake on it's own run method
    public void run(final double rpm) {
        final double validSpeed = MathUtil.clamp(rpm, -5600, 5600);
        goalspeed = validSpeed;
        Intake.set(-validSpeed);
        // Shooter.set(validSpeed);
        Shooter.getClosedLoopController().setSetpoint(validSpeed, ControlType.kVelocity);
        // SmartDashboard.putNumber("Shooter Setpoint", vel_rpm);

    }

    private void configMotor() {
        SparkMaxConfig shooterConfig = new SparkMaxConfig();

        shooterConfig
            // .smartCurrentLimit()
            .inverted(true)
            .idleMode(IdleMode.kCoast);
        shooterConfig.closedLoop
            // .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            // Set PID values for velocity control
            .p(kP)
            .i(0.0)
            .d(0.0)
            .feedForward.kV(kF);
        // intakeConfig.closedLoop.maxMotion
        //     .maxAcceleration(20000, ClosedLoopSlot.kSlot3)  // Note: in m/s^2
        //     .cruiseVelocity(10000, ClosedLoopSlot.kSlot3)  // Note: in m/s
        //     .allowedProfileError(1, ClosedLoopSlot.kSlot3);

        Shooter.configure(shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        SparkMaxConfig intakeConfig = new SparkMaxConfig();

        intakeConfig
            // .smartCurrentLimit()
            .inverted(true)
            .idleMode(IdleMode.kCoast);
        // shooterConfig.closedLoop
        //     // .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        //     // Set PID values for velocity control
        //     .p(kP)
        //     .i(0.0)
        //     .d(0.0)
        //     .feedForward.kV(kF);
        // intakeConfig.closedLoop.maxMotion
        //     .maxAcceleration(20000, ClosedLoopSlot.kSlot3)  // Note: in m/s^2
        //     .cruiseVelocity(10000, ClosedLoopSlot.kSlot3)  // Note: in m/s
        //     .allowedProfileError(1, ClosedLoopSlot.kSlot3);

        Intake.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public boolean atSpeed() {
        return Math.abs(Shooter.getEncoder().getVelocity() - goalspeed) < tolerance;
    }

    public Command standby() {
        return Commands.run(() -> run(idleSpeed), this);
    }

    public Command shoot() {
        return Commands.run(() -> run(shootSpeed), this);
    }

    //TODO: never seems to continue look atr Commands.waitSeconds
    public Command waitForSpeed() {
       return Commands.waitUntil(this::atSpeed);
      // return Commands.waitTime(new ImmutableTime(1, 1, Seconds));
    }

    public Command intake() {
        return Commands.run(() -> run(intakeSpeed), this);
    }

    public Command outtake() {
        return Commands.run(() -> run(outtakeSpeed), this);
    }

    public Command stop() {
        return Commands.runOnce(() -> this.getCurrentCommand().cancel());
    }
}
