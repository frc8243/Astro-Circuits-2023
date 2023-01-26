package frc.robot.commands;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.PIDConstants;
import frc.robot.subsystems.Drivetrain;

// End Imports

public class TurnToTarget extends CommandBase {
    PhotonCamera camera = new PhotonCamera("LIMELIGHT");
    double turnSpeed;
    PIDController turnController;
    private final Drivetrain drivetrain;

    public TurnToTarget(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        turnController = new PIDController(PIDConstants.ANGULAR_P, 0, PIDConstants.ANGULAR_D);
    }

    public void execute() {
        var result = camera.getLatestResult();
        if (result.hasTargets()) {
            System.out.println("Turning to Target!");
            turnSpeed = turnController.calculate(result.getBestTarget().getYaw(), 0);
        } else {
            System.out.println("Not Turning");
            turnSpeed = 0;
        }
        System.out.println(turnSpeed);
        this.drivetrain.m_robotDrive.arcadeDrive(0, turnSpeed);
    }

    public void end(boolean interrupted) {
        System.out.println("Ending Command");
    }
}
