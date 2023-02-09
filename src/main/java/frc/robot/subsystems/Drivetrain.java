package frc.robot.subsystems;

// Imports
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import frc.robot.commands.*;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
// End Imports

public class Drivetrain extends SubsystemBase {

    private final WPI_VictorSPX LB_motor = new WPI_VictorSPX(Constants.DriveConstants.kLeftBack);
    private final WPI_VictorSPX LF_motor = new WPI_VictorSPX(Constants.DriveConstants.kLeftFront);
    private final WPI_VictorSPX RB_motor = new WPI_VictorSPX(Constants.DriveConstants.kRightBack);
    private final WPI_VictorSPX RF_motor = new WPI_VictorSPX(Constants.DriveConstants.kRightFront);
    public final DifferentialDrive m_robotDrive = new DifferentialDrive(LF_motor, RF_motor);

    public Drivetrain() {

        LF_motor.setInverted(false);
        LB_motor.setInverted(false);
        RF_motor.setInverted(true);
        RB_motor.setInverted(true);
        LB_motor.follow(LF_motor);
        RB_motor.follow(RF_motor);

    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("NavX Connected", RobotContainer.ahrs.isConnected());
        SmartDashboard.putNumber("NavX Yaw", RobotContainer.ahrs.getYaw());
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run when in simulation

    }

    public void setMotors(double leftSpeed, double rightSpeed) {
        LF_motor.set(leftSpeed);
        RF_motor.set(rightSpeed);
    }

}
