
package frc.robot;

import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
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
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.XboxConstants;
import frc.robot.Constants.clawConstants;
import frc.robot.commands.arm.LiftyDroppy;
import frc.robot.commands.auton.*;
import frc.robot.commands.claw.SqueezyReleasy;
import frc.robot.commands.drivetrain.CurvatureDrive;
import frc.robot.commands.drivetrain.TurnToTarget;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.NavX;
import frc.robot.subsystems.PID_ProfileArm;
import frc.robot.subsystems.Vision;

public class RobotContainer {
  private static RobotContainer m_robotContainer = new RobotContainer();
  /* Creating global subsystems for the robot to use */
  public static Drivetrain m_drivetrain;
  public static NavX m_navx;
  public static Claw m_claw;
  public static PID_ProfileArm m_arm;
  public static Vision m_vision;
  /* This line allows us to get the data from the PDH/PDP for plots/logging */
  public static PowerDistribution m_pdp;
  /* This line is the sim arm */
  public static MechanismLigament2d armMechanism;
  /* This line creates and assigns our controllers */
  private final XboxController xboxController1 = new XboxController(0);
  private final XboxController xboxController2 = new XboxController(1);
  // Autonomous Position Chooser
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  private RobotContainer() {
    /* I have no idea why (if someone would explain to me if they see this i would appreciate it)
    These subsystems have to be declared in here for them to be static or the robot code crashes */
    m_drivetrain = new Drivetrain();
    m_navx = new NavX();
    m_claw = new Claw();
    m_arm = new PID_ProfileArm();
    m_pdp = new PowerDistribution(11, ModuleType.kRev);
    m_vision = new Vision();
    /* This allows us to access photonvision over USB for when our radio is configured to field */
    PortForwarder.add(5800, "limelight.local", 5800);
    PortForwarder.add(5800, "rasppi.local", 5800);
    /* This section puts data from pdp and m_drivetrain into shuffleboard */
    SmartDashboard.putData(m_pdp);
    SmartDashboard.putData(m_drivetrain);
    configureButtonBindings();

    // Driving Controls
    m_drivetrain.setDefaultCommand(new CurvatureDrive(m_drivetrain,
        // Add a minus ( - ) to either of these to invert the direction the stick has to
        // be pushed : ) - Julien
        () -> xboxController1.getRawAxis(XboxConstants.kLeftStickY),
        () -> -xboxController1.getRawAxis(XboxConstants.kRightStickX)));
    SmartDashboard.putNumber("Controls/Left Stick Y %", xboxController1.getLeftY());
    SmartDashboard.putNumber("Controls/Right Stick X %", -xboxController1.getRightX());
    //-------------------------------------------------------------------------------------
    // Auton Commands
    //-------------------------------------------------------------------------------------
    CommandBase oneConeBalance = new OneConeBalance(m_drivetrain, m_claw, m_arm);
    m_chooser.addOption("oneConeBalance", oneConeBalance);

    CommandBase oneCubeBalance = new OneCubeBalance(m_drivetrain, m_claw, m_arm);
    m_chooser.addOption("oneCubeBalance", oneCubeBalance);
    
    CommandBase oneCone = new OneCone(m_drivetrain, m_claw, m_arm);
    m_chooser.addOption("oneCone", oneCone);

    CommandBase oneConeMove = new OneConeMove(m_drivetrain, m_claw, m_arm);
    m_chooser.addOption("oneConeMove", oneConeMove);

    CommandBase oneCube = new OneCube(m_drivetrain, m_claw, m_arm);
    m_chooser.addOption("oneCube", oneCube);

    CommandBase oneCubeMove = new OneCubeMove(m_drivetrain, m_claw, m_arm);
    m_chooser.addOption("oneCubeMove", oneCubeMove);

    CommandBase driveFowardbalance = new DriveforwardBalance(m_drivetrain, m_claw, m_arm);
    m_chooser.addOption("driveFowardBalance", driveFowardbalance);
    
    CommandBase driveForwardUniBalance = new DriveForwardUniBalance(m_drivetrain, m_claw, m_arm);
    m_chooser.addOption("uniBalance", driveForwardUniBalance);

    CommandBase backUp = new BackUp(m_drivetrain);
    m_chooser.addOption("backUp", backUp);

    SmartDashboard.putData("Auton", m_chooser);

    Mechanism2d mech = new Mechanism2d(1, 1);
    MechanismRoot2d root = mech.getRoot("root", 0.3, 0);
    armMechanism = root.append(new MechanismLigament2d("arm", Constants.ArmConstants.kArmLength, 140));
    armMechanism.setColor(new Color8Bit(222, 28, 28));
    SmartDashboard.putData("Mech2d", mech);

  }

  public static RobotContainer getInstance() {
    return m_robotContainer;
  }

  /* Button Bindings! Buttons are formatted as XboxConstants.(kNameofButton) */
  private void configureButtonBindings() {
    // new JoystickButton(xboxController1, XboxConstants.kAButton).onTrue(
    //     new TurnToTarget(m_drivetrain));

    new JoystickButton(xboxController1, XboxConstants.kAButton).onTrue(
        new TurnToTarget(m_drivetrain));

    new JoystickButton(xboxController2, XboxConstants.kRightBumper).whileTrue(
        new SqueezyReleasy(m_claw, clawConstants.kClawSpeed));

    new JoystickButton(xboxController2, XboxConstants.kLeftBumper).whileTrue(
        new SqueezyReleasy(m_claw, -clawConstants.kClawSpeed));

    new JoystickButton(xboxController1, XboxConstants.kRightStickClick).onTrue(
        new InstantCommand(() -> CurvatureDrive.turnButton = !CurvatureDrive.turnButton));

    new JoystickButton(xboxController1, XboxConstants.kLeftStickClick).onTrue(
        new InstantCommand(() -> CurvatureDrive.isSlow = !CurvatureDrive.isSlow));

    new JoystickButton(xboxController2, XboxConstants.kAButton).onTrue(
        Commands.runOnce(
            () -> {
              m_arm.setGoal(ArmConstants.kArmToFloor);
              m_arm.enable();
              System.out.println("Y Pressed");
            },
            m_arm));

    new JoystickButton(xboxController2, XboxConstants.kYButton).onTrue(
        Commands.runOnce(
            () -> {
              m_arm.setGoal(ArmConstants.kArmLoadingLocation);
              m_arm.enable();
              System.out.println("Start Pressed");
            },
            m_arm));

    new JoystickButton(xboxController2, XboxConstants.kBButton).onTrue(
        Commands.runOnce(
            () -> {
              m_arm.setGoal(ArmConstants.kArmRestingLocation);
              m_arm.enable();
              System.out.println("Back Pressed");
            },
            m_arm));

    new JoystickButton(xboxController2, XboxConstants.kXButton).onTrue(
        Commands.runOnce(
            () -> {
              m_arm.setGoal(ArmConstants.kArmScoringLocation);
              // m_arm.setGoal(100);
              m_arm.enable();
              System.out.println("X Pressed");
            },
            m_arm));
          
    // Manual Arm Constrols
    new POVButton(xboxController1,90).onTrue(
      Commands.runOnce(
        () -> {
          m_arm.disable();
        },
        m_arm));
      
    new POVButton(xboxController1, 0).whileTrue(
      new LiftyDroppy(m_arm, ArmConstants.kArmManualSpeed));
      
    
    new POVButton(xboxController1, 180).whileTrue( 
      new LiftyDroppy(m_arm, -ArmConstants.kArmManualSpeed));
      
    new POVButton(xboxController1, 270).onTrue(
      Commands.runOnce(
        () -> { //Manual Reset for Arm Zero Position
          m_arm.m_encoder.setPosition(0);
          m_arm.setGoal(ArmConstants.kArmRestingLocation);
 
        }
      )
    );
    
  }

  // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=FUNCTIONS
  public XboxController getXboxController1() {
    return xboxController1;
  }
  public XboxController getXboxController2() {
    return xboxController2;
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   * 
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
