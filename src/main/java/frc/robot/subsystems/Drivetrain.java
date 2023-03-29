package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxRelativeEncoder.Type;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;;

public class Drivetrain extends SubsystemBase {

    private final CANSparkMax LBMotor = new CANSparkMax(DriveConstants.kLeftBack,MotorType.kBrushed);
    private final CANSparkMax LFMotor = new CANSparkMax(DriveConstants.kLeftFront,MotorType.kBrushed);
    private final CANSparkMax RBMotor = new CANSparkMax(DriveConstants.kRightBack,MotorType.kBrushed);
    private final CANSparkMax RFMotor = new CANSparkMax(DriveConstants.kRightFront,MotorType.kBrushed);
    public final RelativeEncoder leftEncoder = LFMotor.getEncoder(Type.kQuadrature, 8192);
    public final RelativeEncoder rightEncoder = RFMotor.getEncoder(Type.kQuadrature, 8192);
    public final DifferentialDrive m_robotDrive = new DifferentialDrive(LFMotor, RFMotor);


    public Drivetrain() {
        /* Restore defaults to clear any possible errors*/
        LFMotor.restoreFactoryDefaults();
        RFMotor.restoreFactoryDefaults();
        LBMotor.restoreFactoryDefaults();
        RBMotor.restoreFactoryDefaults();
        /* Left side motors are inverted */
        LFMotor.setInverted(true);
        LBMotor.setInverted(true);
        RFMotor.setInverted(false);
        RBMotor.setInverted(false);
        /* Set rear motor to follow front motor on each side*/
        LBMotor.follow(LFMotor);
        RBMotor.follow(RFMotor);
        /* Robot tries to stay still at all times - Not Sliding */
        LFMotor.setIdleMode(IdleMode.kBrake);
        LBMotor.setIdleMode(IdleMode.kBrake);
        RFMotor.setIdleMode(IdleMode.kBrake);
        RBMotor.setIdleMode(IdleMode.kBrake);
        /* Encoder Shenanigans */
        leftEncoder.setInverted(true);
        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);
        // leftEncoder.setPositionConversionFactor(1/2.07);
        // rightEncoder.setPositionConversionFactor(1/2.07);
        

    }

    @Override
    public void periodic() {
        // SmartDashboard.putNumber("leftSpeed", );
        // SmartDashboard.putNumber("rightSpeed", RBMotor.getMotorOutputPercent());
        // System.out.println("in periodic");
        SmartDashboard.putNumber("Drivetrain/Left Encoder Value", leftEncoder.getPosition());
        SmartDashboard.putNumber("Drivetrain/Right Encoder Value", rightEncoder.getPosition());
        SmartDashboard.putNumber("Drivetrain/Right Front Speed %", RFMotor.get());
        SmartDashboard.putNumber("Drivetrain/Left Front Speed %", LFMotor.get());
        SmartDashboard.putNumber("Drivetrain/Right Back Speed %", RBMotor.get());
        SmartDashboard.putNumber("Drivetrain/Left Back Speed %", LBMotor.get());
        
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run when in simulation

    }

    public void setMotors(double leftSpeed, double rightSpeed) {
        LFMotor.set(leftSpeed);
        RFMotor.set(rightSpeed);
       


    }

}
