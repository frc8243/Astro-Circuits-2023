// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PID_ProfileArm;

public class LiftyDroppy extends CommandBase {
  private final double power;
  private final PID_ProfileArm arm;
  /** Creates a new LiftyDroppy. */
  public LiftyDroppy(PID_ProfileArm armSubsystem, double power) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.arm = armSubsystem;
    this.power= power;
    addRequirements(armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Manual Arm Mode Init");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.arm.disable();
    this.arm.setMotor(power);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.arm.setMotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
