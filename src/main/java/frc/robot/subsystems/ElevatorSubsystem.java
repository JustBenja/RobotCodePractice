package frc.robot.subsystems;
import frc.robot.Constants;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase{
    private final SparkMax motor;
    private final DutyCycleEncoder dutyEncoder;
    private final RelativeEncoder relEncoder;
    private final ElevatorFeedforward ff;

    public ElevatorSubsystem(){
        motor = new SparkMax(Constants.ElevatorConstants.MOTOR_ID, MotorType.kBrushless);

        ff = new ElevatorFeedforward(Constants.ElevatorConstants.kS,
                                     Constants.ElevatorConstants.kG, 
                                     Constants.ElevatorConstants.kV,
                                     Constants.ElevatorConstants.kA);

        SparkMaxConfig config = new SparkMaxConfig();
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        config.closedLoop.pidf(Constants.ElevatorConstants.kP, Constants.ElevatorConstants.kI, Constants.ElevatorConstants.kD, 1.0 / Constants.ElevatorConstants.kV);

        dutyEncoder = new DutyCycleEncoder(Constants.ElevatorConstants.ABS_ENCODER_CHANNEL,
                                           Constants.ElevatorConstants.ABS_ENCODER_RANGE, 
                                           Constants.ElevatorConstants.ABS_ENCODER_OFFSET);

        relEncoder = motor.getEncoder();
        relEncoder.setPosition(dutyEncoder.get());
    }


    // move the motor to a certin angle. (360 degrees is one rotation)
    public void setPosition(double angle){ 
        double current = relEncoder.getPosition() * 360; // current position in degrees
        double targetVelocityDegPerSec = relEncoder.getVelocity() / 60; // current RPS (rounds per second)
        double arbFFVolts = ff.calculate(
            Math.toRadians(current),
            Math.toRadians(targetVelocityDegPerSec)
        );
        motor.getClosedLoopController().setReference(angle, ControlType.kPosition, ClosedLoopSlot.kSlot0, arbFFVolts);
    }

    // give voltage to the motor
    public void set(double voltage){
        double ffVoltage = ff.calculate(Math.toRadians(relEncoder.getPosition() * 360));
        motor.setVoltage(ffVoltage + voltage);
    }
}
