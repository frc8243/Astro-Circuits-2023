package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.kauailabs.navx.frc.*;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;


public class NavX extends SubsystemBase {
    
    public static AHRS ahrs;

    public NavX() {
        ahrs = new AHRS(SPI.Port.kMXP);
        ahrs.zeroYaw();
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("NavX Connected", ahrs.isConnected());
        SmartDashboard.putNumber("NavX Yaw", ahrs.getYaw());
        SmartDashboard.putNumber("NavX Pitch", ahrs.getPitch());
        SmartDashboard.putNumber("NavX Roll", ahrs.getRoll());
    }

    @Override
    public void simulationPeriodic() {

    }
    
}


