package frc.robot.commands;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.PIDConstants;
import frc.robot.subsystems.Drivetrain;

// End Imports

public class TurnToTarget extends CommandBase {
    PhotonCamera camera = new PhotonCamera("OV5647");
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
            turnSpeed = -turnController.calculate(result.getBestTarget().getYaw(), 0);
        } else {
            turnSpeed = 0;
        }
        this.drivetrain.m_robotDrive.arcadeDrive(0, turnSpeed);
    }

    public void end(boolean interrupted) {

    }
}
