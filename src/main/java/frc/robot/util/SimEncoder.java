package frc.robot.util;

import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.SimDevice.Direction;

public class SimEncoder {

  private SimDouble distance;
  private SimDouble speed;

  /**
   * Construct a encoder given a CAN ID.
   *
   * @param name Name of the encoder must be a unique name.
   */

   //we "borrowed" code from Alex 
   //(https://github.com/Mechanisms-Robotics/HawkingBeta/blob/drivetrain/src/main/java/frc/robot/util/SimEncoder.java)
   //and then had to update it bc the original createDouble was "deprecated" (outdated)
   //the only difference was direction instead of the boolean readonly
   //we chose kOutput bc distance is an output 😻
   
   //check if kOutput should be kBidir or kInput 👾
  public SimEncoder(String name) {
    SimDevice device = SimDevice.create("Encoder[" + name + "]");
    distance = device.createDouble("Distance", Direction.kOutput, 0);
    speed = device.createDouble("Speed", Direction.kOutput, 0);
  }
  
  /**
   * Get the speed of the encoder.
   *
   * @return The speed of the encoder in whatever units the user used when
   *     calling setSpeed.
   */
  public double getSpeed() {
    return speed.get();
  }

  /**
   * Get the distance of the encoder.
   *
   * @return The distance of the encoder in whatever units the user used when
   *     calling setDistance.
   */
  public double getDistance() {
    return distance.get();
  }

  /**
   * Set the speed of the encoder.
   *
   * @param speed Speed of the encoder in unit's of the users choice.
   */
  public void setSpeed(double speed) {
    this.speed.set(speed);
  }

  /**
   * Set the distance of the encoder.
   *
   * @param distance Distance of the encoder in unit's of the users choice.
   */
  public void setDistance(double distance) {
    this.distance.set(distance);
  }
}
