// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.filter.SlewRateLimiter;

import edu.wpi.first.wpilibj2.command.*;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;



public class Arm extends SubsystemBase {
  
  SlewRateLimiter filter;
  private final WPI_VictorSPX  Arm_Motor = new WPI_VictorSPX(Constants.ArmConstants.kArm);
  
  private State armState;
 

  /** Creates a new Arm. */
  public Arm() {
    filter = new SlewRateLimiter(ArmConstants.maxSpeed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void setMotors(double power){
    power = filter.calculate(power);
    Arm_Motor.set(power);

  }

  public void moveArmDown(){
    armState = State.MOVING;
    Arm_Motor.set(ArmConstants.ARM_SPEED_DOWN);
    System.out.println("Arm go down");
  }

  public void moveArmUp(){
    armState = State.MOVING;
    Arm_Motor.set(ArmConstants.ARM_SPEED_UP);
    System.out.println("Arm go up");
  }

  public void moveArmScore(){
    if(armState == State.ARM_UP){
      armState = State.MOVING;
      Arm_Motor.set(ArmConstants.ARM_SPEED_DOWN);
      System.out.println("Arm go at angle down");
    }
    else{
      armState = State.MOVING;
      Arm_Motor.set(ArmConstants.ARM_SPEED_UP);
      System.out.println("Arm go at angle up");
    }
    

  }


  public enum State {
    ARM_UP,
    ARM_DOWN,
    ARM_SCORE,
    MOVING
  }
}
