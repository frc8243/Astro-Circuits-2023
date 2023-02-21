// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.BalanceConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.NavX;;


public class AutoBalance extends CommandBase {
    private final Drivetrain drivetrain;
    boolean WITHIN_TARGET_ANGLE = false;
    double speed;
  /** Creates a new AutoBalance. */
  public AutoBalance(Drivetrain drivetrain) {
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("AutoBalance Init");

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    speed = BalanceConstants.kP * NavX.ahrs.getRoll();
    if (speed < 0) {
      speed *= BalanceConstants.REVERSE_POWER_MULTIPLIER;
    }
    if (Math.abs(speed) > 0.4) {
      speed = Math.copySign(0.4, speed);
    }
    // SmartDashboard.putNumber("AutoBalance Speed", speed);
    this.drivetrain.setMotors(speed,speed);
    // this.drivetrain.m_robotDrive.arcadeDrive(speed, 0); 
  }
    
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.drivetrain.setMotors(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //return Math.abs(NavX.ahrs.getRoll()) <= 3.75;
    return false;
  }
}
