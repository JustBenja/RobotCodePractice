package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

public class PivotSubsystem extends SubsystemBase {
    private final SparkMax motor;
    private final RelativeEncoder relEncoder;
    private final DutyCycleEncoder dutyEncoder;
    
    public PivotSubsystem(){
        motor = new SparkMax(Constants.PivotConstants.MOTOR_ID, MotorType.kBrushless);

        relEncoder = motor.getEncoder();

        dutyEncoder = new DutyCycleEncoder(Constants.PivotConstants.ABS_ENCODER_CHANNEL,
                                           Constants.PivotConstants.ABS_ENCODER_RANGE, 
                                           Constants.PivotConstants.ABS_ENCODER_OFFSET);

        
      SparkMaxConfig config = new SparkMaxConfig();
      motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

      config.closedLoop.pid(Constants.PivotConstants.kP, Constants.PivotConstants.kI, Constants.PivotConstants.kD);
        
      relEncoder.setPosition(distance());
        
    }
    
    public void SetPosition(double angle){
        motor.getClosedLoopController().setReference(angle, ControlType.kPosition);
    }

    private double distance(){
        return dutyEncoder.get() * 360;
    }
}
