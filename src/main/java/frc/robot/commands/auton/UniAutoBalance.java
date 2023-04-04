// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.drivetrain.ArcadeDrive;
import frc.robot.subsystems.Drivetrain;


// This command self=balances on the charging station using gyroscope pitch as feedback
public class UniAutoBalance extends CommandBase {

  private Drivetrain drivetrain;

  private double drivePower;

  /** Command to use Gyro data to resist the tip angle from the beam - to stabalize and balanace */
  public UniAutoBalance(Drivetrain drivetrain) {
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    drivePower = 0.01 * drivetrain.getGyroPitch();
    drivePower = Math.max(Math.min(drivePower, 0.3), -0.3);
    drivePower = -drivePower;

    if (Math.abs(drivetrain.getGyroPitchRate()) > 15) {
      drivePower = 0;
    }
  
   //converting drivePower into voltage 
    drivePower *= 12; 
    this.drivetrain.setMotors(drivePower, drivePower);

  }

  @Override
  public void end(boolean interrupted) {
    this.drivetrain.setMotors(0, 0);
  }

  @Override
  public boolean isFinished() {
   // return Math.abs(error) < Constants.DriveConstants.BEAM_BALANCED_ANGLE_TRESHOLD_DEGREES; // End the command when we are within the specified threshold of being 'flat' (gyroscope pitch of 0 degrees)
    return false;
  }
}
