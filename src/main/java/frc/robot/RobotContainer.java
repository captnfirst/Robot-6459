package frc.robot;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Head;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Turret;

public class RobotContainer {
  Arm m_arm = new Arm();
  Head m_head = new Head();
  Drive m_drive = new Drive();
  Intake m_intake = new Intake();
  // Elevator m_elevator = new Elevator();
  Turret m_turret = new Turret(m_drive);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
  }

  public Command getAutonomousCommand() {

    // ****** First try successful ********* //
    PathPlannerTrajectory firstPath = PathPlanner.loadPath("New New Path", new PathConstraints(2, 1));

    RamseteCommand ramseteCommand = new RamseteCommand(
        firstPath,
        m_drive::getPose,
        new RamseteController(2, .7),
        new SimpleMotorFeedforward(
            0.18442,
            2.4541,
            0.31906),
        DriveConstants.kDriveKinematics,
        m_drive::getWheelSpeeds,
        new PIDController(0.52215, 0, 0), // 0.10445
        new PIDController(0.52215, 0, 0), // 0.11595
        // RamseteCommand passes volts to the callback
        m_drive::tankDriveVolts,
        m_drive);

    // Reset odometry to the starting pose of the trajectory.
    m_drive.resetOdometry(firstPath.getInitialPose());

    // Run path following command, then stop at the end.
    return ramseteCommand.andThen(() -> m_drive.tankDriveVolts(0, 0));
  }
}
