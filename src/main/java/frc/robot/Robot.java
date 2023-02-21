package frc.robot;

import java.io.IOException;
import java.nio.file.Path;

import com.pathplanner.lib.server.PathPlannerServer;

// Imports
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

// End Imports

public class Robot extends TimedRobot {

    private Command m_autonomousCommand;
    private RobotContainer m_robotContainer;

    //Path Planner things :(
    String trajectoryJSON = "pathplanner/generatedJSON/Path1.wpilib.json";
    Trajectory trajectory = new Trajectory();

    @Override
    public void robotInit() {
        // This starts up the robot - Button bindings and imports Autonomous code
        m_robotContainer = RobotContainer.getInstance();
        HAL.report(tResourceType.kResourceType_Framework, tInstances.kFramework_RobotBuilder);
  // =----------[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[00000000000000000000]]';;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;']]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]    PathPlannerTrajectory examplePath = PathPlanner.loadPath("Example Path", new PathConstrains(4, 3));
        PathPlannerServer.startServer(5811); // 5811 = port number. adjust this according to your needs

        // More Path Planner whatever :)
        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
            trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
         } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
         }
    }

    /**
    * This function is called every robot packet, no matter the mode. Use this for items like
    * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
    *
    * <p>This runs after the mode specific periodic functions, but before
    * LiveWindow and SmartDashboard integrated updating.
    */
    @Override
    public void robotPeriodic() {
        // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
        // commands, running already-scheduled commands, removing finished or interrupted commands,
        // and running subsystem periodic() methods.  This must be called from the robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run();
    }


    /**
    * This function is called once each time the robot enters Disabled mode.
    */
    @Override
    public void disabledInit() {
        m_robotContainer.disabledInit();
    }

    @Override
    public void disabledPeriodic() {
    }

    /**
    * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
    */
    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        // schedule the autonomous command (example)
        if (m_autonomousCommand != null) {

System.out.println("paper box");
            m_autonomousCommand.schedule();
        }
    }

    /**
    * This function is called periodically during autonomous.
    */
    @Override
    public void autonomousPeriodic() {


    }

    @Override
    public void teleopInit() {
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
        else {
            m_robotContainer.enabledInit();
        }
    }

    /**
     * This function is called periodically during operator control.
     */
    @Override
    public void teleopPeriodic() {

       // if (Controller.getRawButton(A_BUTTON)){
            //do something
        //}
    }

    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }

    /**
    * This function is called periodically during test mode.
    */
    @Override
    public void testPeriodic() {
      
    }

}