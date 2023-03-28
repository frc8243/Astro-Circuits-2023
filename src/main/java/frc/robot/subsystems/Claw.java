// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Claw extends SubsystemBase {
  SlewRateLimiter filter;

  private CANSparkMax clawMotor = new CANSparkMax(9, MotorType.kBrushed);
  public DigitalInput clawLimitOut = new DigitalInput(0);
  public DigitalInput clawLimitIn = new DigitalInput(1);
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
    SmartDashboard.putNumber("Claw/Power %", clawMotor.get());
    SmartDashboard.putBoolean("Claw/Limit Out", clawLimitOut.get());
    SmartDashboard.putBoolean("Claw/Limit In", clawLimitIn.get());
  }



}

