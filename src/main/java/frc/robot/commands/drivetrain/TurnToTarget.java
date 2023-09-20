package frc.robot.commands.drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
        Vision.LLtoggleLights();
    }

    @Override
    public void execute() {
        // System.out.println(LocalTime.now()); Use this to find loop speed >: )
        var result = Vision.limelight.getLatestResult();
        if (result.hasTargets()) {
            turnSpeed = turnController.calculate(result.getBestTarget().getYaw(), 0);
            SmartDashboard.putNumber("Turn Speed", turnSpeed);
            if (Math.abs(turnSpeed) <= 0.001) {
                withinTargetRange = true;
            } 
            else {
                withinTargetRange = false;
            }
        } 
        else {
            System.out.println("Target Lost");
            turnSpeed = 0;
            withinTargetRange = true;
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
