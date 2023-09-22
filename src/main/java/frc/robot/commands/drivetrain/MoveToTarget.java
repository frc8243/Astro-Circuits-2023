package frc.robot.commands.drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Drivetrain;


// End Imports

public class MoveToTarget extends CommandBase {
    double forwardSpeed;
    PIDController moveController;
    double distanceToTarget;
    private final Drivetrain drivetrain;

    public MoveToTarget(Drivetrain drivetrain, double distanceToTarget) {
        this.drivetrain = drivetrain;
        this.distanceToTarget = distanceToTarget;
        addRequirements(drivetrain);
    } 

    @Override
    public void initialize() {
        moveController = new PIDController(VisionConstants.kMovingP, 0, VisionConstants.kMovingD);
    }

    public void execute() {
        // var result = Vision.limelight.getLatestResult();
        // if (result.hasTargets()) {
        //     System.out.println("Distance to target = " + distanceToTarget);
        //     forwardSpeed = moveController.calculate(result.getBestTarget().getBestCameraToTarget().getX(), distanceToTarget);
        //     System.out.println("forward speed = " + forwardSpeed);
        // } 
        // else {
        //     forwardSpeed = 0;
        // }
        // this.drivetrain.m_robotDrive.curvatureDrive(forwardSpeed,0,true);
    }

    public void end(boolean interrupted) {
        this.drivetrain.m_robotDrive.curvatureDrive(0,0,false);
    }
}
