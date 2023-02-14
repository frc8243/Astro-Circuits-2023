// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Arm extends SubsystemBase {
  
  private CANSparkMax armMotor = new CANSparkMax(4, MotorType.kBrushless);
  /** Creates a new Arm. */
  public Arm() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
