// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Superstructure;

public class Poses extends CommandBase {
  /** Creates a new StartPose. */
  Superstructure m_super;
  double arm;
  double head;
  double turret;
  boolean enable;
  boolean visionON;
  double intakespeed;
  Value value;

  double waitTime;
  double endTime;
  Timer waitTimer = new Timer();
  Timer endTimer = new Timer();

  public Poses(Superstructure m__super, double arm, double head, double turret, double intakespeed,
      Value intakeseloneid,
      boolean visionenabled,
      boolean visionON,
      double waitTime, double endTime) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.arm = arm;
    this.head = head;
    this.turret = turret;
    this.m_super = m__super;
    this.enable = visionenabled;
    this.visionON = visionON;
    this.waitTime = waitTime;
    this.endTime = endTime;
    this.intakespeed = intakespeed;
    this.value = intakeseloneid;
    addRequirements(m_super);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    waitTimer.reset();
    endTimer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    endTimer.start();
    waitTimer.start();
    m_super.lockOnTarget(enable, visionON);
    if (enable == false && visionON == false) {
      m_super.setTurret(true, turret);
    }
    m_super.setIntake(intakespeed);
    m_super.setIntakeSolenoid(value);
    if (waitTimer.get() >= waitTime) {
      m_super.setArm(true, arm);
      m_super.setHead(true, head);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean valueToReturn;
    if (endTimer.get() >= endTime) {
      valueToReturn = true;
    } else {
      valueToReturn = false;
    }
    return valueToReturn;
  }
}
