package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.RobotContainer;
import frc.robot.Constants.ArmConstants;
import frc.robot.util.SimEncoder;

/** A robot arm subsystem that moves with a motion profile. */

public class PID_ProfileArm extends ProfiledPIDSubsystem {

  private CANSparkMax armMotor = new CANSparkMax(ArmConstants.kArmMotor, MotorType.kBrushless);
  private final RelativeEncoder m_encoder = armMotor.getEncoder();
  private final ArmFeedforward m_feedforward = new ArmFeedforward(
      ArmConstants.kSVolts, ArmConstants.kGVolts,
      ArmConstants.kVVoltSecondPerRad, ArmConstants.kAVoltSecondSquaredPerRad);

  private SimEncoder armEncoderSim;
  private SingleJointedArmSim armSim;

  /** Create a new ArmSubsystem. */

  public PID_ProfileArm() {
    super(
        new ProfiledPIDController(
            ArmConstants.kP,
            0,
            0,
            new TrapezoidProfile.Constraints(
                ArmConstants.kMaxVelocityRadPerSecond,
                ArmConstants.kMaxAccelerationRadPerSecSquared)),
        1);
        
    armMotor.restoreFactoryDefaults();
    armMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    armMotor.setInverted(true);
    m_encoder.setPosition(0);
    m_encoder.setPositionConversionFactor(82.43);

    // Degrees for one rotation of motor
    m_encoder.setPositionConversionFactor(ArmConstants.kRotationDegree);
    // Start arm at rest in neutral position
    setGoal(ArmConstants.kArmOffsetRads);

    if (RobotBase.isSimulation()) {
      armEncoderSim = new SimEncoder("Elevator");
      armSim = new SingleJointedArmSim(
          DCMotor.getNEO(1), // 1 NEO motor on the climber
          ArmConstants.kGearRatio, // TODO find out gearing
          SingleJointedArmSim.estimateMOI(ArmConstants.kArmLength, ArmConstants.kArmMass),
          ArmConstants.kArmLength,
          ArmConstants.kArmMinAngle,
          ArmConstants.kArmMaxAngle,
          ArmConstants.kArmMass,
          true);
    }
  }

    @Override
    public void simulationPeriodic() {
      // sets input for elevator motor in simulation
      armSim.setInput(armMotor.get() * RobotController.getBatteryVoltage());
      
      // Next, we update it. The standard loop time is 20ms.
      armSim.update(0.02);
      // Finally, we set our simulated encoder's readings
      armEncoderSim.setDistance(armSim.getAngleRads());
      // sets our simulated encoder speeds
      armEncoderSim.setSpeed(armSim.getVelocityRadPerSec());
  
      SmartDashboard.putNumber("voltage", RobotController.getBatteryVoltage());
      SmartDashboard.putNumber("arm angle", armSim.getAngleRads()); 
      SmartDashboard.putNumber("armMotor", armMotor.get());
      // SimBattery estimates loaded battery voltages
      RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(armSim.getCurrentDrawAmps()));
      RobotContainer.armMechanism.setAngle(Units.radiansToDegrees(armSim.getAngleRads()));
  @Override
  public void simulationPeriodic() {
    // sets input for elevator motor in simulation
    armSim.setInput(armMotor.get() * RobotController.getBatteryVoltage());

    // Next, we update it. The standard loop time is 20ms.
    armSim.update(0.02);
    // Finally, we set our simulated encoder's readings
    armEncoderSim.setDistance(armSim.getAngleRads());
    // sets our simulated encoder speeds
    armEncoderSim.setSpeed(armSim.getVelocityRadPerSec());

    SmartDashboard.putNumber("arm angle", armSim.getAngleRads());
    SmartDashboard.putNumber("armMotor", armMotor.get());
    // SimBattery estimates loaded battery voltages
    RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(armSim.getCurrentDrawAmps()));
    RobotContainer.armMechanism.setAngle(Units.radiansToDegrees(armSim.getAngleRads()));

  }

  @Override
  public void periodic() {

  }

  @Override
  public void useOutput(double output, TrapezoidProfile.State setpoint) {
    // Calculate the feedforward from the sepoint
    double feedforward = m_feedforward.calculate(setpoint.position, setpoint.velocity);
    // Add the feedforward to the PID output to get the motor output
    armMotor.setVoltage(output + feedforward);
    System.out.println("useOutput has been called" + output + "< output" + feedforward + "< feed forward");
  }

  @Override
  public double getMeasurement() {
    System.out.println("getMeasure has been called" + m_encoder.getPosition());
    return m_encoder.getPosition();

  }
}