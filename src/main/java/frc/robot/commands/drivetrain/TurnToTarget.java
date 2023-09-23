package frc.robot.commands.drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;

// End Imports

public class TurnToTarget extends CommandBase {
    double turnSpeed;
    PIDController turnController;
    private final Drivetrain drivetrain;
    boolean withinTargetRange = false;

    public TurnToTarget(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);

    }

    @Override
    public void initialize() {
        System.out.println("TurnToTarget Init");
        turnController = new PIDController(VisionConstants.kTurningP, VisionConstants.kTurningI, VisionConstants.kTurningD);
        turnController.setTolerance(1);
        Vision.LLtoggleLights();
    }

    @Override
    public void execute() {
        // System.out.println(LocalTime.now()); Use this to find loop speed >: )
        if (Vision.targetFound == 1.0) {
            turnSpeed = turnController.calculate(Vision.x, 0.0);
        }
        else {
            System.out.println("Target not found");
        }
        if (Math.abs(turnSpeed) >= 0.2) {
            turnSpeed = Math.copySign(0.2, turnSpeed);
        }
        this.drivetrain.m_robotDrive.curvatureDrive(0, turnSpeed, true);
    }

    @Override
    public boolean isFinished() {
        return withinTargetRange;

    }

    @Override
    public void end(boolean interrupted) {
        this.drivetrain.m_robotDrive.curvatureDrive(0, 0, false);
        Vision.LLtoggleLights();
    }

}
