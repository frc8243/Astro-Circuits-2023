// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;

public class SqueezyReleasy extends CommandBase {
  /** Creates a new SqueezyReleasy. */

  private final double power;
  private final Claw claw;
  DigitalInput limit = new DigitalInput(0);

  public SqueezyReleasy(Claw clawSubsystem, double power) {
    this.claw = clawSubsystem;
    this.power = power;
    addRequirements(clawSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Claw cmd init");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(!limit.get()){
      this.claw.setMotor(power);
      System.out.println("Claw motor power : " + power);
    }
    else{
      System.out.println("Claw has hit max");
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.claw.setMotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
