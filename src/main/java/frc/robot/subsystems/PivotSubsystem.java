package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class PivotSubsystem extends SubsystemBase {
    private final SparkMax motor;
    private final RelativeEncoder relEncoder;
    private final DutyCycleEncoder dutyEncoder;
    private final SparkClosedLoopController config;
    
    public PivotSubsystem(){
        motor = new SparkMax(Constants.PivotConstants.MOTOR_ID, MotorType.kBrushless);

        relEncoder = motor.getEncoder();

        dutyEncoder = new DutyCycleEncoder(Constants.PivotConstants.ABS_ENCODER_CHANNEL,
                                           Constants.PivotConstants.ABS_ENCODER_RANGE, 
                                           Constants.PivotConstants.ABS_ENCODER_OFFSET);

        relEncoder.setPosition(distance());
        
        config = motor.getClosedLoopController();
    }
    
    public void SetPosition(double angle){
        relEncoder.setPosition(angleToRotations(angle) + distance());
        config.setReference(angle, ControlType.kPosition);
    }

    public double angleToRotations(double angle){
        return angle / Constants.PivotConstants.FULL_ROTATION_IN_ANGELS;
    }

    public double distance(){
        return dutyEncoder.get() * 360;
    }
}
