package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import static edu.wpi.first.units.Units.Seconds;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;

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
    private static final double shootSpeed = 4000; // 1
    private static final double intakeSpeed = .8; // 0.8
    private static final double outtakeSpeed = -0.8; // -0.8
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
    public void periodic(){
        SmartDashboard.putBoolean("atSpeed", atSpeed());
        SmartDashboard.putNumber("Shootervel", Shooter.getEncoder().getVelocity());
        SmartDashboard.putNumber("Intakevel",Intake.getEncoder().getVelocity());
    }
    



    //TODO: need to have intake on it's own run method
    public void runShooter(final double rpm) {
        final double validSpeed = MathUtil.clamp(rpm, -5600, 5600);
        goalspeed = validSpeed;
        Shooter.getClosedLoopController().setSetpoint(validSpeed, ControlType.kVelocity);
       // Shooter.set(validSpeed);
    }
        public void stopmotors() {
        Shooter.getClosedLoopController().setSetpoint(0, ControlType.kVelocity);
        Intake.set(0);
      
    }
    

    public void runIntake(final double rpm) {
        final double validSpeed = MathUtil.clamp(rpm, -1, 1);
        goalspeed = validSpeed;
        Intake.set(-validSpeed);
        Shooter.set(validSpeed);
    }

    public void stopIntake() {
        Intake.stopMotor();
    }
    private void configMotor(){
        SparkMaxConfig shooterConfig = new SparkMaxConfig();
        
        shooterConfig
            .idleMode(IdleMode.kCoast)
            .smartCurrentLimit(40)
            .inverted(false);
        shooterConfig.closedLoop
            .p(kP)
            .i(0.0)
            .d(0.0)
            .feedForward.kV(kF);
         
        Shooter.configure(shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        SparkMaxConfig intakeConfig = new SparkMaxConfig();
        
        intakeConfig
            .idleMode(IdleMode.kCoast)
            .smartCurrentLimit(40)
            .inverted(true);
   
        Intake.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }
    

    public boolean atSpeed() {
        return Math.abs(Shooter.getEncoder().getVelocity() - goalspeed) < tolerance;
    }

    public Command standby() {
        return Commands.run(() -> stopmotors(), this);
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
        return Commands.runOnce(() -> this.getCurrentCommand().cancel());
    }
}
