package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Turret;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;

  AddressableLED led = new AddressableLED(0);
  AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(78);
  int id;
  int b;
  int hue;
  Timer ledBlinkTimer = new Timer();
  double blinkTime = 0.1;

  Joystick driverJoystick = new Joystick(0);
  Joystick yardimciJoystick = new Joystick(1);

  Timer backlashTimer = new Timer();
  Timer teleoptime = new Timer();
  Timer human = new Timer();

  double kDriveSpeed = 0.87;
  double kRotationSpeed = 0.56;

  int desiredPos = 0; // -1 = human / 0 = start / 1 = 1 lvl / 2 = 2 lvl / 3 = 3 lvl / 4 = cube ground
  int gamePiece = 0; // -1 = cube / 0 = Rainbow / 1 = cone // 2 = ReadyToShoot // 3 = last 20 sec
  boolean autoResetStatus = false;

  double armAngleOffset_humanPlayer = 0;
  double headAngleOffset_lowerPole = 0;
  double armAngleOffset_lowerPole = 0;
  double armAngleOffset_lower = 0;

  Timer downTimer = new Timer();
  double downOffset = 0;
  double downAngle = -10;

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    m_robotContainer.m_drive.gyro.reset();
    CameraServer.startAutomaticCapture();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    setLeds(gamePiece);
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
    gamePiece = 0;
    m_robotContainer.m_intake.setIntakeSolenoid(Value.kOff);
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
    autoResetStatus = true;
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
    // teleoptime.start();
    // if (teleoptime.get() > 125) {
    // gamePiece = 3; // balance red
    // }

    m_robotContainer.m_drive.drive.arcadeDrive(driverJoystick.getRawAxis(1) * kDriveSpeed,
        driverJoystick.getRawAxis(2) * kRotationSpeed);
    if (desiredPos == -1) { // human player
      kDriveSpeed = 0.6;
      kRotationSpeed = 0.53;
    } else if (desiredPos == 0) { // start pos
      if (m_robotContainer.m_arm.getArmPos() < 20) {
        kDriveSpeed = 0.87;
        kRotationSpeed = 0.56;
      }
    } else if (desiredPos == 1) { // 1lvl
      kDriveSpeed = 0.78;
      kRotationSpeed = 0.54;
    } else if (desiredPos == 2 && desiredPos == 3) { // 2lvl / 3lvl cube
      kDriveSpeed = 0.55;
      kRotationSpeed = 0.53;
    } else if (desiredPos == 4) { // cube ground
      kDriveSpeed = 0.5;
      kRotationSpeed = 0.53;
    }

    if (yardimciJoystick.getRawButton(2)) {
      desiredPos = 0; // start
    } else if (yardimciJoystick.getRawButton(1)) {
      desiredPos = -1; // human
    } else if (yardimciJoystick.getRawButton(4)) {
      desiredPos = 1; // 1 lvl
    } else if (yardimciJoystick.getRawButton(3)) {
      desiredPos = 2; // 2 lvl
    } else if (yardimciJoystick.getRawButton(12)) {
      desiredPos = 3; // 3lvl cube
    } else if (yardimciJoystick.getRawButton(11)) {
      desiredPos = 4; // cube ground
    }

    if (yardimciJoystick.getPOV() == 0) {
      if (desiredPos == -1) {
        armAngleOffset_humanPlayer += 0.1;
      }
    } else if (yardimciJoystick.getPOV() == 180) {
      if (desiredPos == -1) {
        armAngleOffset_humanPlayer -= 0.1;
      }
    }

    if (yardimciJoystick.getRawButton(5)) {
      downTimer.start();
      if (desiredPos == -1) { // human
        m_robotContainer.m_intake.setIntakeSolenoid(Value.kReverse);
      } else if (desiredPos == 0) { // start
        m_robotContainer.m_intake.setIntakeSolenoid(Value.kReverse);
      } else if (desiredPos == 2 && gamePiece == 1) { // 2lvl cone
        if (downTimer.get() < 0.5) {
          downOffset = downAngle;
        } else {
          m_robotContainer.m_intake.setIntakeSolenoid(Value.kReverse);
          if (downTimer.get() > 0.7) {
            downOffset = 0;
          }
        }
      } else if (desiredPos == 2 && gamePiece == -1) { // 2lvl cube
        m_robotContainer.m_intake.setIntake(-0.7);
      } else if (desiredPos == 1 && gamePiece == 1) { // 1lvl cone
        m_robotContainer.m_intake.setIntakeSolenoid(Value.kReverse);
      } else if (desiredPos == 1 && gamePiece == -1) { // 1lvl cube
        if (m_robotContainer.m_turret.getTurretPos() > -40 && m_robotContainer.m_turret.getTurretPos() < 40) {
          m_robotContainer.m_intake.intakeMotor.stopMotor();
        } else {
          m_robotContainer.m_intake.setIntake(-1);
        }
      } else if (desiredPos == 3) {
        m_robotContainer.m_intake.setIntake(-0.7);
      } else if (desiredPos == 4) { // ground cube
        m_robotContainer.m_intake.setIntakeSolenoid(Value.kReverse);
      }
    } else if (yardimciJoystick.getRawButton(6)) {
      if (desiredPos == -1) { // human
        human.start();
        if (human.get() > 0.2) {
          m_robotContainer.m_intake.setIntake(0.1);
        } else {
          m_robotContainer.m_intake.intakeMotor.stopMotor();
        }
        if (gamePiece == 1) { // cone
          m_robotContainer.m_intake.setIntakeSolenoid(Value.kForward);
        } else if (gamePiece == -1) { // cube
          m_robotContainer.m_intake.setIntakeSolenoid(Value.kOff);
        } else { // other
          m_robotContainer.m_intake.setIntakeSolenoid(Value.kOff);
        }
      } else if (desiredPos == 4) { // ground cube
        m_robotContainer.m_intake.setIntake(0.2);
      } else if (desiredPos == 1 && gamePiece == -1) { // 1lvl cube
        m_robotContainer.m_intake.setIntakeSolenoid(Value.kOff);
      } else if (desiredPos == 2 && gamePiece == -1) { // 2lvl cube
        m_robotContainer.m_intake.setIntakeSolenoid(Value.kOff);
      } else if (desiredPos == 3) { // 3lvl cube
        m_robotContainer.m_intake.setIntakeSolenoid(Value.kOff);
      } else { // other
        m_robotContainer.m_intake.setIntakeSolenoid(Value.kForward);
        downTimer.reset();
        downOffset = 0;
      }
    } else { // button don't push
      m_robotContainer.m_intake.intakeMotor.stopMotor();
      downTimer.reset();
      downOffset = 0;
    }

    if (yardimciJoystick.getRawButton(10)) {
      m_robotContainer.m_drive.gyro.reset();
    }

    if (yardimciJoystick.getRawButton(7) && (desiredPos == 2)) {
      m_robotContainer.m_turret.visionLockingStatus = true;
      if (Turret.targetStatus == 1 && Turret.x > -1 && Turret.x < 1) {
        gamePiece = 2; // ready
      } else {
        gamePiece = 1; // cone
      }
    } else {
      // m_robotContainer.m_turret.visionLockingStatus = false;
      // m_robotContainer.m_turret.visionIncrement = 0;
      if (gamePiece == 2) { // ready
        gamePiece = 1; // cone
      }
    }

    if (driverJoystick.getRawButton(5)) {
      gamePiece = 1; // cone
    } else if (driverJoystick.getRawButton(6)) {
      gamePiece = -1; // cube
    }

    backlashTimer.start();
    if (backlashTimer.get() > 2) {
      if (desiredPos == -1) { // human
        m_robotContainer.m_turret.setTurret(true, 180);
        if (m_robotContainer.m_turret.getTurretPos() > 170 && m_robotContainer.m_turret.getTurretPos() < 190) {
          m_robotContainer.m_arm.setArm(true, 97.5 + armAngleOffset_humanPlayer);
          if (m_robotContainer.m_arm.getArmPos() > 30) {
            m_robotContainer.m_head.setHead(true, 4000);
          }
        }
        m_robotContainer.m_turret.lockOnTarget(false, false);
      } else if (desiredPos == 0) { // start
        m_robotContainer.m_arm.setArm(true, -4);
        m_robotContainer.m_head.setHead(true, 12000);
        if (m_robotContainer.m_arm.getArmPos() < 20) {
          m_robotContainer.m_turret.setTurret(true, 0);
        }
        m_robotContainer.m_turret.lockOnTarget(false, false);
      } else if (desiredPos == 1) { // 1lvl
        double cubedOffset;
        double cubedArmOffset;
        if (gamePiece == -1) {
          if (m_robotContainer.m_arm.getArmPos() < 40) {
            cubedOffset = 2000;
            cubedArmOffset = -36;
          } else {
            cubedOffset = 0;
            cubedArmOffset = 0;
          }
        } else {
          cubedOffset = 0;
          cubedArmOffset = 0;
        }
        m_robotContainer.m_arm.setArm(true, 32 + cubedArmOffset + armAngleOffset_lower);
        m_robotContainer.m_turret.setTurret(true,
            (m_robotContainer.m_turret.getTurretAbs() - m_robotContainer.m_turret.getTurningOffset()));
        if (m_robotContainer.m_arm.getArmPos() > 0) {
          m_robotContainer.m_head.setHead(true, 8500 + cubedOffset); // 6000
        }
        m_robotContainer.m_turret.lockOnTarget(false, false);
      } else if (desiredPos == 2) { // 2lvl
        double cubeOffset;
        double cubeArmOffset;
        if (gamePiece == -1) {
          if (m_robotContainer.m_arm.getArmPos() > 30) {
            cubeOffset = -18800;
            cubeArmOffset = 18;
          } else {
            cubeOffset = 0;
            cubeArmOffset = 0;
          }
        } else {
          cubeOffset = 0;
          cubeArmOffset = 0;
        }
        m_robotContainer.m_arm.setArm(true, 100.5 + downOffset + armAngleOffset_lowerPole + cubeArmOffset);
        if (m_robotContainer.m_arm.getArmPos() > 40) {
          m_robotContainer.m_turret.lockOnTarget(true, m_robotContainer.m_turret.visionLockingStatus);
          m_robotContainer.m_head.setHead(true, 2500 + headAngleOffset_lowerPole + cubeOffset);
        }
      } else if (desiredPos == 3) {
        // m_robotContainer.m_turret.lockOnTarget(true,
        // m_robotContainer.m_turret.visionLockingStatus);
        m_robotContainer.m_arm.setArm(true, 120);
        if (m_robotContainer.m_arm.getArmPos() > 30) {
          m_robotContainer.m_head.setHead(true, -1000);
        }
      } else if (desiredPos == 4) { // ground cube
        if (m_robotContainer.m_arm.getArmPos() > 40) {
          m_robotContainer.m_arm.setArm(true, 22.5);
          m_robotContainer.m_head.setHead(true, -2500);
          if (m_robotContainer.m_arm.getArmPos() < 25) {
            m_robotContainer.m_turret.setTurret(true, 180);
          }
        } else {
          m_robotContainer.m_turret.setTurret(true, 180);
          if (m_robotContainer.m_turret.getTurretPos() > 170 && m_robotContainer.m_turret.getTurretPos() < 190) {
            m_robotContainer.m_arm.setArm(true, 22.5);
            if (m_robotContainer.m_arm.getArmPos() < 25) {
              m_robotContainer.m_head.setHead(true, -2500);
            }
          }
        }
        m_robotContainer.m_turret.lockOnTarget(false, false);
      }
    } else if (backlashTimer.get() < 1.95) { // system back run
      m_robotContainer.m_head.headMotor.set(ControlMode.PercentOutput, 0.05);
      m_robotContainer.m_arm.armMotor.set(ControlMode.PercentOutput, -0.05);
      m_robotContainer.m_head.headMotor.setSelectedSensorPosition(0);
      m_robotContainer.m_arm.armMotor.setSelectedSensorPosition(0);
    } else {
      m_robotContainer.m_head.headMotor.stopMotor();
      m_robotContainer.m_arm.armMotor.stopMotor();
    }
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {

  }

  @Override
  public void simulationInit() {
  }

  @Override
  public void simulationPeriodic() {
  }

  void huePlus() {
    hue++;
    if (hue == 180) {
      hue = 0;
    } else {
      ledBuffer.setHSV(id, hue, 255, 255);
    }
  }

  void idPlus() {
    id++;
    if (id == 78) {
      id = 0;
    } else {
      ledBuffer.setHSV(id, hue, 255, 255);
    }
  }

  public void setLeds(int ledStatus) {
    Alliance ally = DriverStation.getAlliance();
    ledBlinkTimer.start();
    if (ledStatus == 0) { // rainbow
      idPlus();
      huePlus();
    } else if (ledStatus == -1) { // purple cube
      if (ledBlinkTimer.get() < blinkTime) {
        for (int i = 0; i < 78; i++) {
          if (i > 42 && i < 79) {
            ledBuffer.setRGB(i, 255, 0, 255);
          } else if (i < 42 && ally.equals(Alliance.Red)) {
            ledBuffer.setRGB(i, 255, 0, 0);
          } else {
            ledBuffer.setRGB(id, 0, 0, 255);
          }
        }
      } else if (ledBlinkTimer.get() > blinkTime && ledBlinkTimer.get() < blinkTime * 2) {
        for (int i = 0; i < 78; i++) {
          ledBuffer.setRGB(i, 0, 0, 0);
        }
      } else {
        ledBlinkTimer.reset();
      }
    } else if (ledStatus == 1) { // yellow cone
      if (ledBlinkTimer.get() < blinkTime) {
        for (int i = 0; i < 78; i++) {
          if (i > 42 && i < 79) {
            ledBuffer.setRGB(i, 255, 255, 0);
          } else if (i < 42) {
            if (ally.equals(Alliance.Red)) {
              ledBuffer.setRGB(i, 255, 0, 0);
            } else if (ally.equals(Alliance.Blue)) {
              ledBuffer.setRGB(id, 0, 0, 255);
            }
          }
        }
      } else if (ledBlinkTimer.get() > blinkTime && ledBlinkTimer.get() < blinkTime * 2) {
        for (int i = 0; i < 78; i++) {
          ledBuffer.setRGB(i, 0, 0, 0);
        }
      } else {
        ledBlinkTimer.reset();
      }
    } else if (ledStatus == 2) { // green ready
      if (ledBlinkTimer.get() < blinkTime) {
        for (int i = 0; i < 78; i++) {
          if (i > 42 && i < 79) {
            ledBuffer.setRGB(i, 0, 255, 0);
          } else if (i < 42) {
            if (ally.equals(Alliance.Red)) {
              ledBuffer.setRGB(i, 255, 0, 0);
            } else if (ally.equals(Alliance.Blue)) {
              ledBuffer.setRGB(id, 0, 0, 255);
            }
          }
        }
      } else if (ledBlinkTimer.get() > blinkTime && ledBlinkTimer.get() < blinkTime * 2) {
        for (int i = 0; i < 78; i++) {
          ledBuffer.setRGB(i, 0, 0, 0);
        }
      } else {
        ledBlinkTimer.reset();
      }
    } else if (ledStatus == 3) { // red balance
      if (ledBlinkTimer.get() < blinkTime) {
        for (int i = 0; i < 78; i++) {
          if (i > 42 && i < 79) {
            ledBuffer.setRGB(i, 255, 0, 0);
          } else if (i < 42) {
            if (ally.equals(Alliance.Red)) {
              ledBuffer.setRGB(i, 255, 0, 0);
            } else if (ally.equals(Alliance.Blue)) {
              ledBuffer.setRGB(id, 0, 0, 255);
            }
          }
        }
      } else if (ledBlinkTimer.get() > blinkTime && ledBlinkTimer.get() < blinkTime * 2) {
        for (int i = 0; i < 78; i++) {
          ledBuffer.setRGB(i, 0, 0, 0);
        }
      } else {
        ledBlinkTimer.reset();
      }
    }
    led.setLength(78);
    led.setData(ledBuffer);
    led.start();
  }
}