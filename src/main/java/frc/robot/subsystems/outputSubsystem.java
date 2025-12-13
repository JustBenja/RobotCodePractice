// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class outputSubsystem extends SubsystemBase {
    private final String SubsystemMsg;

  /** Creates a new ExampleSubsystem. */
  public outputSubsystem() {
    this.SubsystemMsg = "hi chat this is me the real sponja :)";
  }

  public void printMsg(){
    System.out.println(SubsystemMsg);
  }
}
