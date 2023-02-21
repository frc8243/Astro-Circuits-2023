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
    boolean WITHIN_TARGET_RANGE = false;

    public TurnToTarget(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);

    }

    @Override
    public void initialize() {
        System.out.println("TurnToTarget Init");
        turnController = new PIDController(PIDConstants.ANGULAR_P, PIDConstants.ANGULAR_I , PIDConstants.ANGULAR_D);
    }

    @Override
    public void execute() {
        // System.out.println(LocalTime.now()); Use this to find loop speed >: )
        var result = camera.getLatestResult();
        if (result.hasTargets()) {
            turnSpeed = turnController.calculate(result.getBestTarget().getYaw(), 0);
            System.out.println(turnSpeed + " Turn Speed");
            if (Math.abs(turnSpeed) <= 0.001) {
                WITHIN_TARGET_RANGE = true;
            } else {
                WITHIN_TARGET_RANGE = false;
            }
        } else {
            System.out.println("Target Lost");
            turnSpeed = 0;
            WITHIN_TARGET_RANGE = true;
        }
        this.drivetrain.m_robotDrive.arcadeDrive(0, turnSpeed);
    }

    @Override
    public boolean isFinished() {
        return WITHIN_TARGET_RANGE;

    }

    @Override
    public void end(boolean interrupted) {
        this.drivetrain.m_robotDrive.arcadeDrive(0, 0);

    }

}
