// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.clawConstants;
import frc.robot.commands.claw.SqueezyReleasy;
import frc.robot.commands.drivetrain.DriveForwardGivenDistanceUsingTime;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.PID_ProfileArm;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class OneConeBalance extends SequentialCommandGroup {
  /** Creates a new OnePieceBalance. */
  public OneConeBalance(Drivetrain drivetrain, Claw claw, PID_ProfileArm arm) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand( // Sets arm down to score
          () -> {
            arm.setGoal(ArmConstants.kArmScoringLocation);
            arm.enable();
            // System.out.println("Back Pressed");
            },
          arm),
      new WaitUntilCommand(() -> arm.atGoal()).withTimeout(4),
      new WaitCommand(1.5),
      new SqueezyReleasy(claw, -clawConstants.kClawSpeed).withTimeout(0.75),
      // new PrintCommand("Arm reached resting position"),
      new DriveForwardGivenDistanceUsingTime(-3 , drivetrain), //This number is in meters
      new PrintCommand("Got past Drive Forward Given Distnace"),
      //new DriveForwardGivenDistanceUsingTime(1.5 , drivetrain),
      new AutoBalance(drivetrain)
      // new InstantCommand( // Sets arm down to score
      // () -> {
      //   arm.setGoal(ArmConstants.kArmRestingLocation);
      //   arm.enable();
      //   // System.out.println("Back Pressed");
      //   },
      // arm)
      ); 
  }
}