package frc.robot.subsystems;

import java.net.CacheRequest;

import com.ctre.phoenix.motorcontrol.NeutralMode;
// Imports
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxRelativeEncoder.Type;

import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {

    private final CANSparkMax LB_motor = new CANSparkMax(Constants.DriveConstants.kLeftBack, MotorType.kBrushed);
    private final CANSparkMax LF_motor = new CANSparkMax(Constants.DriveConstants.kLeftFront,MotorType.kBrushed);
    private final CANSparkMax RB_motor = new CANSparkMax(Constants.DriveConstants.kRightBack,MotorType.kBrushed);
    private final CANSparkMax RF_motor = new CANSparkMax(Constants.DriveConstants.kRightFront,MotorType.kBrushed);
    public final RelativeEncoder leftEncoder = LF_motor.getEncoder(Type.kQuadrature, 8192);
    public final RelativeEncoder rightEncoder = RF_motor.getEncoder(Type.kQuadrature, 8192);
    public final DifferentialDrive m_robotDrive = new DifferentialDrive(LF_motor, RF_motor);

    public Drivetrain() {

        LF_motor.setInverted(true);
        LB_motor.setInverted(true);
        RF_motor.setInverted(false);
        RB_motor.setInverted(false);
        LB_motor.follow(LF_motor);
        RB_motor.follow(RF_motor);
        LF_motor.setIdleMode(IdleMode.kBrake);
        LB_motor.setIdleMode(IdleMode.kBrake);
        RF_motor.setIdleMode(IdleMode.kBrake);
        RB_motor.setIdleMode(IdleMode.kBrake);
        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);
        leftEncoder.setPositionConversionFactor(0);
        rightEncoder.setPositionConversionFactor(0);
        

    }

    @Override
    public void periodic() {
        // SmartDashboard.putNumber("leftSpeed", );
        // SmartDashboard.putNumber("rightSpeed", RB_motor.getMotorOutputPercent());
        // System.out.println("in periodic");
        SmartDashboard.putNumber("Left Encoder Value", leftEncoder.getPosition());
        SmartDashboard.putNumber("Right Encoder Value", rightEncoder.getPosition());

        
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
