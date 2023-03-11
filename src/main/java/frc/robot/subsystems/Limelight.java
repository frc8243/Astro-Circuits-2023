package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.GenericHID;
//import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import frc.robot.commands.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.VisionConstants;


public class Limelight extends SubsystemBase {

  public static final GenericEntry amountTargets = Shuffleboard.getTab("Driver").add("Num of Targets", 0).getEntry();
  public static final GenericEntry node_x_entry = Shuffleboard.getTab("Driver").add("Node X", 0).getEntry();
  public static final GenericEntry node_y_entry = Shuffleboard.getTab("Driver").add("Node Y", 0).getEntry();
  public static final GenericEntry node_distance_entry = Shuffleboard.getTab("Driver").add("Node dist", 0).getEntry();
  public static final GenericEntry node_angle_entry = Shuffleboard.getTab("Driver").add("Node angle", 0).getEntry();
  public static final GenericEntry in_line_entry = Shuffleboard.getTab("Driver").add("Node Aligned", false).getEntry();
  static PhotonCamera camera;
  public static double distanceToNode;
  public static double angleToNode;
  public static boolean has_targets;

  public static boolean withinRangeRumble = false;
  public static boolean ledEnabled = false;
  
  Boolean photonNotFoundMessagePrinted = false ;

  public Limelight() {

    camera = new PhotonCamera("LIMELIGHT");

  }

  @Override
  public void periodic() {

    var result = camera.getLatestResult();

    has_targets = result.hasTargets();

    SmartDashboard.putBoolean("Photon Limelight hasTargets: ", has_targets);
    if (has_targets) {

        // List<PhotonTrackedTarget> targets = result.getTargets();
        PhotonTrackedTarget best_target = result.getBestTarget();

        double distance_to_target = PhotonUtils.calculateDistanceToTargetMeters(
          VisionConstants.CAMERA_HEIGHT_METERS,
          VisionConstants.TARGET_HEIGHT_METERS,
          Units.degreesToRadians(VisionConstants.CAMERA_PITCH_RADIANS),
          Units.degreesToRadians(best_target.getPitch())
        );


        Translation2d translation = PhotonUtils.estimateCameraToTargetTranslation(
          distance_to_target, Rotation2d.fromDegrees(-best_target.getYaw()));


        double x_translation = translation.getX();
        double y_translation = translation.getY();

        node_x_entry.setDouble(x_translation);
        node_y_entry.setDouble(y_translation);
        double node_dist = Math.hypot(x_translation, y_translation);
        angleToNode = Units.radiansToDegrees(Math.atan2(y_translation, x_translation));
        node_distance_entry.setDouble(node_dist);
        node_angle_entry.setDouble(angleToNode);
        int countTargets = result.getTargets().size();
        amountTargets.setDouble(countTargets);
        boolean isInLine = Math.abs(angleToNode) < VisionConstants.kAngleToleranceDegrees;
        if(withinRangeRumble && isInLine ){
          RobotContainer.xboxController1.setRumble(GenericHID.RumbleType.kLeftRumble, 1);
          try {
            Thread.sleep(2);
          } catch (InterruptedException e) {

          }
          RobotContainer.xboxController1.setRumble(GenericHID.RumbleType.kLeftRumble, 0);
          
          

        }
        in_line_entry.setBoolean(isInLine);
        

    }
    else {
      node_x_entry.setDouble(0);
      node_y_entry.setDouble(0);
      node_distance_entry.setDouble(0);
      node_angle_entry.setDouble(0);
      amountTargets.setDouble(0);
      in_line_entry.setBoolean(false);
      angleToNode = 0;

    }
  }


  public static double getAngleToNode() {
    return angleToNode;
  }

  

  @Override
  public void simulationPeriodic() {
    //This method will be called once per scheduler run when in simulation

  }



  public static void turnLEDOn() {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);
  }

  public static void turnLEDOff() {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
  }

}
