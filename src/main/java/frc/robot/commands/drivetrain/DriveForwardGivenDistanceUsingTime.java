// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class DriveForwardGivenDistanceUsingTime extends CommandBase {

  Timer timer;
  static double SPEED = 2/3.;
  double time;
  Drivetrain m_driveTrain;
  double distance;

  public DriveForwardGivenDistanceUsingTime(double distance, Drivetrain subsystem) {
      // initial variables
      m_driveTrain = subsystem;
      addRequirements(m_driveTrain);
      time = Math.abs(distance * .425); /* This number is 1/V */ //TODO : Make this t = d/v
      this.distance = distance;
      // set command to be interuptible
      // setInterruptible(true);

      timer = new Timer();

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

      timer.reset();
      timer.start();
      addRequirements(m_driveTrain);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //   m_driveTrain.setMotors(Math.copySign(SPEED, distance),Math.copySign(SPEED, distance));
    this.m_driveTrain.m_robotDrive.curvatureDrive(Math.copySign(SPEED, distance), 0, false);

      

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // m_driveTrain.setMotors(0, 0);
    this.m_driveTrain.m_robotDrive.curvatureDrive(0, 0, false);


  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
      return timer.get() >= time;
  }

  @Override
  public boolean runsWhenDisabled() {
      return false;

  }
 
}
