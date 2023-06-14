package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
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
  //Elevator m_elevator = new Elevator();
  Turret m_turret = new Turret(m_drive);
  
  SendableChooser<Command> autoChooser = new SendableChooser<>();

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
  }

  public Command getAutonomousCommand() {
    // return new Autonomous(m_drive);
    return null;
  }
}
