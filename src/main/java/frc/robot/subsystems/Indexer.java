package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase {
    private static final int motorID = 13;
    private static final double inSpeed = 1;
    private static final double outSpeed = -1;
    private static final double idleSpeed = 0;

    private final SparkMax motor;

    //TODO: Add motor configurations
    public Indexer() {
        super("Indexer");

        motor = new SparkMax(motorID, MotorType.kBrushless);

        setDefaultCommand(idle());
    }
    
    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putNumber("IndexerVel", motor.getEncoder().getVelocity());
    }

    public void run(final double speed) {
        final double validSpeed = MathUtil.clamp(speed, -1, 1);

        motor.set(validSpeed);
    }

    public Command idle() {
        return Commands.run(() -> run(idleSpeed), this);
    }

    public Command in() {
        return Commands.run(() -> run(inSpeed), this);
    }

    public Command out() {
        return Commands.run(() -> run(outSpeed), this);
    }

    //TODO: this seems to cause the code to crash
    public Command stop() {
        return Commands.runOnce(() -> this.getCurrentCommand().cancel(), this);
    }
}
