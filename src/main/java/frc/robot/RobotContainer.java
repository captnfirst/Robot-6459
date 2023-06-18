package frc.robot;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.Poses;
import frc.robot.subsystems.Superstructure;

public class RobotContainer {
        public Superstructure m_super = new Superstructure();

        public RobotContainer() {
                configureBindings();
        }

        private void configureBindings() {
        }

        public Command getAutonomousCommand() {

                // ****** First try successful ********* //
                PathPlannerTrajectory firstPath = PathPlanner.loadPath("first", new PathConstraints(2, 2));
                PathPlannerTrajectory secondPath = PathPlanner.loadPath("second", new PathConstraints(2, 2));
                PathPlannerTrajectory thirdPath = PathPlanner.loadPath("third", new PathConstraints(3, 3));
                PathPlannerTrajectory fourPath = PathPlanner.loadPath("four", new PathConstraints(3, 3));

                RamseteCommand red1_1 = new RamseteCommand(
                                firstPath,
                                m_super::getPose,
                                new RamseteController(2, .7),
                                new SimpleMotorFeedforward(
                                                0.18442,
                                                2.4541,
                                                0.31906),
                                DriveConstants.kDriveKinematics,
                                m_super::getWheelSpeeds,
                                new PIDController(0.52215, 0, 0), // 0.10445
                                new PIDController(0.52215, 0, 0), // 0.11595
                                // red1_1 passes volts to the callback
                                m_super::tankDriveVolts,
                                m_super);

                RamseteCommand red1_2 = new RamseteCommand(
                                secondPath,
                                m_super::getPose,
                                new RamseteController(2, .7),
                                new SimpleMotorFeedforward(
                                                0.18442,
                                                2.4541,
                                                0.31906),
                                DriveConstants.kDriveKinematics,
                                m_super::getWheelSpeeds,
                                new PIDController(0.52215, 0, 0), // 0.10445
                                new PIDController(0.52215, 0, 0), // 0.11595
                                // red1_2 passes volts to the callback
                                m_super::tankDriveVolts,
                                m_super);

                RamseteCommand red1_3 = new RamseteCommand(
                                thirdPath,
                                m_super::getPose,
                                new RamseteController(2, .7),
                                new SimpleMotorFeedforward(
                                                0.18442,
                                                2.4541,
                                                0.31906),
                                DriveConstants.kDriveKinematics,
                                m_super::getWheelSpeeds,
                                new PIDController(0.52215, 0, 0), // 0.10445
                                new PIDController(0.52215, 0, 0), // 0.11595
                                // red1_3 passes volts to the callback
                                m_super::tankDriveVolts,
                                m_super);

                RamseteCommand red1_4 = new RamseteCommand(
                                fourPath,
                                m_super::getPose,
                                new RamseteController(2, .7),
                                new SimpleMotorFeedforward(
                                                0.18442,
                                                2.4541,
                                                0.31906),
                                DriveConstants.kDriveKinematics,
                                m_super::getWheelSpeeds,
                                new PIDController(0.52215, 0, 0), // 0.10445
                                new PIDController(0.52215, 0, 0), // 0.11595
                                // red1_4 passes volts to the callback
                                m_super::tankDriveVolts,
                                m_super);

                SequentialCommandGroup firstGroup = new SequentialCommandGroup(
                        new InstantCommand(() -> {m_super.resetBacklash(true);}),
                        new Poses(m_super, 0, 10000, 0, 0, Value.kOff, false, false, 0.255, 1),
                        new Poses(m_super, 0, 10000, 0, 0, Value.kOff, false, false, 0, 0.3),
                        new Poses(m_super, 100.5, 2500, 0, 0, Value.kOff, true, true, 0, 1.2),
                        red1_1.andThen(new Poses(m_super, 95.5, 2000, 0, 0, Value.kOff, true, true, 0, 1.3)),
                        new Poses(m_super, 95.5, 2000, 0, 0, Value.kReverse, true, true, 0, 1.3),
                        new Poses(m_super, 0, 10000, 0, 0, Value.kOff, false, false, 0, 1),
                        new Poses(m_super, 0, 10000, 0, 0, Value.kOff, false, false, 0, 0.5),
                        new Poses(m_super, 0, 10000, 180, 0, Value.kOff, false, false, 0, 1.2),
                        new Poses(m_super, 0, -2500, 180, 0.3, Value.kOff, false, false, 0, 1.2),
                        red1_2.andThen(new Poses(m_super, 0, -2500, 180, 0.3, Value.kOff, false, false, 0,1.2)));

                SequentialCommandGroup mainGroup = new SequentialCommandGroup(firstGroup);

                // Reset odometry to the starting pose of the trajectory.
                m_super.resetOdometry(firstPath.getInitialPose());

                return mainGroup;
        }
}
