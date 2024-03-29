// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.claw;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;

public class SqueezyReleasy extends CommandBase {
  /** Creates a new SqueezyReleasy. */

  private final double power;
  private final Claw claw;

  public SqueezyReleasy(Claw clawSubsystem, double power) {
    this.claw = clawSubsystem;
    // clawLimit = new DigitalInput(0);
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
    if (power < 0) {
      if (claw.clawLimitOut.get()) {
        this.claw.setMotor(power);
      } 
      else {
        this.claw.setMotor(0);
        System.out.println("Claw is at maximum");
      }
    } 
    else if (power > 0) {
      if (claw.clawLimitIn.get()) {
        this.claw.setMotor(power);
      } 
      else {
        this.claw.setMotor(0);
        System.out.println("Claw is at minimum");
      }
    } 
    else {
      /* This should never happen, power would have to be zero when command is called */
      System.out.println("Something is horribly wrong.");
    }

    System.out.println("Claw motor power : " + power);

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
