package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.Drivetrain;

public class Autonomous extends CommandBase {

    double turnPercent, forwardPercent;

    Timer timer;
    double sp;
    double tm;
    Drivetrain m_driveTrain;

    public Autonomous(double speed, double time, Drivetrain subsystem) {
        // initial variables
        sp = speed;
        tm = time;
        m_driveTrain = subsystem;
        addRequirements(m_driveTrain);

        // set command to be interuptible
        // setInterruptible(true);

        timer = new Timer();

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

        timer.reset();
        timer.start();
        addRequirements(m_driveTrain);

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_driveTrain.setMotors(sp, -sp);

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return timer.get() >= tm;
    }

    @Override
    public boolean runsWhenDisabled() {
        return false;

    }
}
