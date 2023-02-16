// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.CAN;

public class Claw extends SubsystemBase {
  SlewRateLimiter filter;

  private CANSparkMax clawMotor = new CANSparkMax(12, MotorType.kBrushed);
  /** Creates a new Claw. */
  public Claw() {
  }

  public void setMotor(double power){
    clawMotor.set(power);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }



}

