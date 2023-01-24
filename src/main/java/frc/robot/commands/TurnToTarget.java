package frc.robot.commands;


// Imports
import java.util.function.Supplier;
import org.photonvision.PhotonCamera;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.PIDConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.commands.ArcadeDrive;
import edu.wpi.first.math.controller.PIDController;

// End Imports

public class TurnToTarget extends CommandBase {
    PhotonCamera camera = new PhotonCamera("OV5647");
    double turnSpeed;
    PIDController turnController = new PIDController(PIDConstants.ANGULAR_P, 0, PIDConstants.ANGULAR_D);

    @Override
    public void initialize() {
 
    }

    public void execute() {
        var result = camera.getLatestResult();
        if (result.hasTargets()) {
            turnSpeed = -turnController.calculate(result.getBestTarget().getYaw(), 0);
        }
        else;
            turnSpeed = 0;
    }

    public void end(boolean interrupted) {

    }
}
