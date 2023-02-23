// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class SimpleArmMovement extends CommandBase {
  private final Arm armSubsystem;
  private final double power;
  

  /** Creates a new SimpleArmMovement. */
  public SimpleArmMovement(Arm armSubsystem, double power) {
    this.armSubsystem = armSubsystem;
    this.power = power;
    addRequirements(armSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.armSubsystem.setMotor(power);
    System.out.println("Arm is moving at > " + power);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Arm is stopped");
    this.armSubsystem.setMotor(Math.copySign(0.025, power));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
