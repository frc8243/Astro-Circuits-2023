// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.*;
import org.photonvision.common.hardware.VisionLEDMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
  /** Creates a new Limelight. */
  public final static PhotonCamera limelight = new PhotonCamera("limelight");
  public final static PhotonCamera frontCam = new PhotonCamera("frontCam"); 
  public final static PhotonCamera backCam = new PhotonCamera("backCam"); 

  public Vision() {
    limelight.setLED(VisionLEDMode.kOff);
    
  }

  @Override
  public void periodic()
  {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Vision/Limelight Connected",limelight.isConnected());
    SmartDashboard.putBoolean("Vision/RaspPi Connected",frontCam.isConnected());

  }

  public static void LLtoggleLights() {
    if (limelight.getLEDMode() == VisionLEDMode.kOn) {
      limelight.setLED(VisionLEDMode.kOff);
    }
    else if (limelight.getLEDMode() == VisionLEDMode.kOff) {
      limelight.setLED(VisionLEDMode.kOn);
    }
   
  }

  public static void toggleDriverMode() {
    
  }


}
