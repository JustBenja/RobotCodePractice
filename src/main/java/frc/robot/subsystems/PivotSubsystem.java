package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.*;

public class PivotSubsystem extends SubsystemBase {
    SparkMax motor = new SparkMax(0, MotorType.kBrushless);
    
}
