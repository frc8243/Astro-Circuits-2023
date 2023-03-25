// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class DriveForwardGivenDistance extends CommandBase {
  /** Creates a new DriveForwardGivenDistance. */

  double currentPosition;
  double targetDistanceMeters;
  double newTargetDistance;
  Drivetrain m_Drivetrain;
  private SlewRateLimiter driveLimiter;
  public DriveForwardGivenDistance(double targetDistanceMeters, Drivetrain subsystem) {

    this.targetDistanceMeters = targetDistanceMeters;
    m_Drivetrain = subsystem;
    addRequirements(m_Drivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    currentPosition = (
      m_Drivetrain.leftEncoder.getPosition() +
      m_Drivetrain.rightEncoder.getPosition());
    
    System.out.println("starting current position" + currentPosition);

    newTargetDistance  = currentPosition + targetDistanceMeters;

    this.driveLimiter  = new SlewRateLimiter(0.75);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    currentPosition = (m_Drivetrain.leftEncoder.getPosition() + m_Drivetrain.rightEncoder.getPosition())/2;

    double error = newTargetDistance - currentPosition;

    double outputSpeed = (1 * error);
    outputSpeed = MathUtil.clamp(outputSpeed, -0.3, 0.3);
    outputSpeed = driveLimiter.calculate(outputSpeed);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Drivetrain.setMotors(0, 0);

    System.out.println("DriveForwardGIvenDistance ended");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double errorTolerance = 0.01;

    if(Math.abs(newTargetDistance - currentPosition) <= errorTolerance){
      return true;
    }
    return false;
  }
}
