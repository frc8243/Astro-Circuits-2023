// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
  /** Creates a new Arm. */
  private CANSparkMax armMotor = new CANSparkMax(14, MotorType.kBrushless);
  private RelativeEncoder armEncoder = armMotor.getEncoder();
  public double target;

  public Arm() {
    armMotor.restoreFactoryDefaults();
    armMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    armMotor.setInverted(true);
    armEncoder.setPosition(0);
    armEncoder.setPositionConversionFactor(82.43);
  }

  public void setMotor(double power) {
    armMotor.set(power);
  }

  public double getPosition(){
    return armEncoder.getPosition();
    
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
