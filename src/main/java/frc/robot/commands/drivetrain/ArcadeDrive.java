package frc.robot.commands;

//Imports
import java.util.function.Supplier;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drivetrain;
//End Imports

public class ArcadeDrive extends CommandBase {

    private final Drivetrain drivetrain;
    private final Supplier<Double> forwardSpeed, turnSpeed;
    public static boolean isSlow = false;

    public ArcadeDrive(Drivetrain drivetrain, Supplier<Double> forwardSpeed, Supplier<Double> turnSpeed) {
        this.drivetrain = drivetrain;
        this.forwardSpeed = forwardSpeed;
        this.turnSpeed = turnSpeed;
        addRequirements(drivetrain);
       
    }

    // Runs when ArcadeDrive is initialized
    @Override
    public void initialize() {
        System.out.println("init ArcadeDrive");
        isSlow = true;
    }

    // Called
    @Override
    public void execute() {
        // System.out.println(forwardSpeed.get() + " forward " + turnSpeed.get() + " turn ");
  

        if(isSlow){
            this.drivetrain.m_robotDrive.arcadeDrive(forwardSpeed.get() * DriveConstants.SLOW_SPEED_FRACTION, turnSpeed.get() * DriveConstants.SLOW_SPEED_FRACTION);
        }
        else{
            this.drivetrain.m_robotDrive.arcadeDrive(forwardSpeed.get(), turnSpeed.get());
        }

    }



    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public boolean runsWhenDisabled() {
        return false;
    }
}
