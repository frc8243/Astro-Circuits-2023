// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.clawConstants;

public class Claw extends SubsystemBase {
  SlewRateLimiter filter;

  private CANSparkMax clawMotor = new CANSparkMax(clawConstants.kClawMotor, MotorType.kBrushed);
  public DigitalInput clawLimitOut = new DigitalInput(clawConstants.kOutLimitSwitch);
  public DigitalInput clawLimitIn = new DigitalInput(clawConstants.kInLimitSwitch);
  /** Creates a new Claw. */
  public Claw() {
    clawMotor.restoreFactoryDefaults();
    clawMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
  }
 
  public void setMotor(double power){
    clawMotor.set(power);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }



}

