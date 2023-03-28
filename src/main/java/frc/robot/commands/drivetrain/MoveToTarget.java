package frc.robot.commands;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.PIDConstants;
import frc.robot.subsystems.Drivetrain;


// End Imports

public class MoveToTarget extends CommandBase {
    PhotonCamera camera = new PhotonCamera("LIMELIGHT");
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
        moveController = new PIDController(PIDConstants.LINEAR_P, 0, PIDConstants.LINEAR_D);
    }

    public void execute() {
        var result = camera.getLatestResult();
        if (result.hasTargets()) {
            System.out.println("Distance to target = " + distanceToTarget);
            
            forwardSpeed = moveController.calculate(result.getBestTarget().getBestCameraToTarget().getX(), distanceToTarget);
            System.out.println("forawrd speed = " + forwardSpeed);
        } else {
            forwardSpeed = 0;
        }
        this.drivetrain.m_robotDrive.arcadeDrive(forwardSpeed,0);
    }

    public void end(boolean interrupted) {
        this.drivetrain.m_robotDrive.arcadeDrive(0, 0);



    }
}
