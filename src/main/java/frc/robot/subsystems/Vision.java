// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
  /** Creates a new Limelight. */
  public static PhotonCamera frontCam;
  public static PhotonCamera armCam;
  static NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tv = table.getEntry("tv");
  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry ty = table.getEntry("ty");
  NetworkTableEntry ta = table.getEntry("ta");
  public static double targetFound;
  public static double x;
  public static double y;
  public static double area;

  public Vision() {
    frontCam = new PhotonCamera("frontCam");
    armCam = new PhotonCamera("armCam");
    System.out.println("Vision Init");
    LLtoggleLights();
    armCam.setDriverMode(true);
    frontCam.setDriverMode(true);
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Vision/RaspPi Connected",frontCam.isConnected());
    targetFound = tv.getDouble(0.0);
    x = tx.getDouble(0.0);
    y = ty.getDouble(0.0);
    area = ta.getDouble(0.0);
    SmartDashboard.putNumber("Vision/Target Found", targetFound);
    SmartDashboard.putNumber("Vision/Target X Error", x);
    SmartDashboard.putNumber("Vision/Target Y Error", y);
    SmartDashboard.putNumber("Vision/Target Area", area);
  }

  public static void LLtoggleLights() {
    if (table.getEntry("ledMode").getDouble(0.0) == 1.0) {
      table.getEntry("ledMode").setNumber(3.0);
    }
    else if (table.getEntry("ledMode").getDouble(0.0) == 3.0 || table.getEntry("ledMode").getDouble(0.0) == 0) {
      table.getEntry("ledMode").setNumber(1.0);
    }
  }

  public static void toggleDriverMode() {
    
  }


}
