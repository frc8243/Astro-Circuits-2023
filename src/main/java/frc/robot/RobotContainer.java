
package frc.robot;

import org.photonvision.PhotonCamera;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.XboxConstants;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.ArmMovement;
import frc.robot.commands.AutoBalance;
import frc.robot.commands.Autonomous;
import frc.robot.commands.MoveToTarget;
import frc.robot.commands.SqueezyReleasy;
import frc.robot.commands.TurnToTarget;
import frc.robot.commands.doNothingArm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.NavX;
import frc.robot.subsystems.PID_ProfileArm;

public class RobotContainer {
  private static RobotContainer m_robotContainer = new RobotContainer();
  // The robot's subsystems
  public final Drivetrain m_drivetrain = new Drivetrain();
  public final NavX m_navx = new NavX();
  public final Claw m_claw = new Claw();
  public final PID_ProfileArm m_arm = new PID_ProfileArm();


  public static MechanismLigament2d armMechanism;
  
  // Declaring Controller

  private final XboxController xboxController1 = new XboxController(0);
  PhotonCamera camera = new PhotonCamera("OV5647");
  // Autonomous Position Chooser
  SendableChooser<Command> m_chooser = new SendableChooser<>();


  



  private RobotContainer() {
    // Smartdashboard Subsystemsnull, nul
    SmartDashboard.putData(m_drivetrain);
    configureButtonBindings();
  
    // Driving Controls
    m_drivetrain.setDefaultCommand(new ArcadeDrive(m_drivetrain,
      //Add a minus ( - ) to either of these to invert the direction the stick has to be pushed : ) - Julien
        () -> -xboxController1.getRawAxis(XboxConstants.LEFT_STICK_Y),
        () -> -xboxController1.getRawAxis(XboxConstants.RIGHT_STICK_X)
        ));

    m_arm.setDefaultCommand(new doNothingArm(m_arm));

    CommandBase position1 = new SequentialCommandGroup(
        new Autonomous(-0.25, 4, m_drivetrain),
        new Autonomous(0, 10, m_drivetrain));
        m_chooser.addOption("position1", position1);

    CommandBase balanceTesting = new SequentialCommandGroup(
      new Autonomous(0.5, 1.5, m_drivetrain),
      new AutoBalance(m_drivetrain));
      m_chooser.addOption("balanceTesting", balanceTesting);

    SmartDashboard.putData("Auton", m_chooser);

    Mechanism2d mech = new Mechanism2d(1, 1);
    MechanismRoot2d root = mech.getRoot("root", 0.3, 0);
    armMechanism = root.append(new MechanismLigament2d("arm", Constants.ArmConstants.kArmLength, 140));
    armMechanism.setColor(new Color8Bit(222, 28, 28));
    SmartDashboard.putData("Mech2d", mech);
  
    // try {
    //   CameraServer.startAutomaticCapture(1);
    // } catch (Exception ex1) {
    //   System.out.println("Camera not found");
    // }
    // CameraServer.startAutomaticCapture();
    }

  public static RobotContainer getInstance() {
    return m_robotContainer;
  }

  /**
   * Button bindings go below
   */
  private void configureButtonBindings() {
    new JoystickButton(xboxController1, XboxConstants.LEFT_BUMPER).onTrue(new TurnToTarget(m_drivetrain));
    new JoystickButton(xboxController1, XboxConstants.RIGHT_BUMPER).onTrue(new MoveToTarget(m_drivetrain,1));
    new JoystickButton(xboxController1, XboxConstants.A_BUTTON).onTrue(new AutoBalance(m_drivetrain));
    new JoystickButton(xboxController1, XboxConstants.X_BUTTON).whileTrue(new SqueezyReleasy(m_claw, .9));
    new JoystickButton(xboxController1, XboxConstants.B_BUTTON).whileTrue(new SqueezyReleasy(m_claw, -.9));
    new JoystickButton(xboxController1, XboxConstants.Y_BUTTON).onTrue(
        Commands.runOnce(
            () -> {
              m_arm.setGoal(ArmConstants.kArmStraightUp);
              m_arm.enable();
              System.out.println("Y Pressed");
            },
            m_arm));
            

    new JoystickButton(xboxController1, XboxConstants.START_BUTTON).onTrue(
      Commands.runOnce(
        () -> {
          m_arm.setGoal(ArmConstants.kArmLoadingLocation);
          m_arm.enable();
          System.out.println("Start Pressed");
        },
        m_arm));  

    new JoystickButton(xboxController1, XboxConstants.BACK_BUTTON).onTrue(
      Commands.runOnce(
        () -> {
          m_arm.setGoal(ArmConstants.kArmRestingLocation);
          m_arm.enable();
          System.out.println("Start Pressed");
        },
        m_arm));
    //fix slow mode off a button
    new JoystickButton(xboxController1,XboxConstants.Y_BUTTON).onTrue(new InstantCommand(() -> ArcadeDrive.isSlow = !ArcadeDrive.isSlow)); 
  }

 

  // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=FUNCTIONS
  public XboxController getXboxController1() {
    return xboxController1; 
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // The selected command will be run in autonomous
    return m_chooser.getSelected();
  }

  public void disabledInit() {
  }

  public void enabledInit() {
  }
}
