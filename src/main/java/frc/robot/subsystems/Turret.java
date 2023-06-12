// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Turret extends SubsystemBase {
  /** Creates a new Turret. */
  public double turretKConv = -1055.55;
  public double turretMinPos = -548000; // 240000
  public double turretMaxPos = 548000; // -240000

  public WPI_TalonFX turretMotor = new WPI_TalonFX(7);

  private Drive drive;
  public boolean visionLockingStatus;
  public boolean gyroOffsetLock = false;
  public double gyroOffset = 0;
  public double visionIncrement = 0;
  public double minXLimiter = -20;
  public double maxXLimiter = 20;

  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry ty = table.getEntry("ty");
  NetworkTableEntry tv = table.getEntry("tv");
  NetworkTableEntry ta = table.getEntry("ta");
  public static double x = 0;
  public static double y = 0;
  public static double targetStatus = 0;
  public static double targetArea = 0;
  double minTargetArea = 0;

  public Turret(Drive m_drive) {
    this.drive = m_drive;
    turretMotor.configFactoryDefault();
    turretMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
    turretMotor.configNeutralDeadband(0.001, 0);
    turretMotor.setSelectedSensorPosition(0);
    turretMotor.configAllowableClosedloopError(0, 50, 1);
    turretMotor.config_kP(0, 0.1);
    turretMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, 0);
    turretMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, 0);
    turretMotor.configNominalOutputForward(0, 0);
    turretMotor.configNominalOutputReverse(0, 0);
    turretMotor.configPeakOutputForward(1, 0);
    turretMotor.configPeakOutputReverse(-1, 0);
    turretMotor.configMotionCruiseVelocity(45000, 0); // 30000
    turretMotor.configMotionAcceleration(30000, 0); // 15000
  }

  public double getTurretPos() {
    double turretPos = (turretMotor.getSelectedSensorPosition() + 0.001) / turretKConv;
    return turretPos;
  }

  public void setTurret(boolean enabled, double turretDegrees) {
    double calculatedAngle = turretKConv * turretDegrees;
    if (enabled) {
      turretMotor.set(ControlMode.MotionMagic, MathUtil.clamp(calculatedAngle, turretMinPos, turretMaxPos));
      if (calculatedAngle > turretMaxPos) {
        System.out.println("Turret MAX is trying to get out of bounds!");
      } else if (calculatedAngle < turretMinPos) {
        System.out.println("Turret MIN is trying to get out of bounds!");
      }
    } else {
      turretMotor.stopMotor();
    }
  }

  public double getXVal() {
    double valueToReturn;
    double kMinimizer = 0.05;
    if (x > minXLimiter && x < maxXLimiter) {
      valueToReturn = x;
    } else {
      valueToReturn = 0;
    }
    valueToReturn = valueToReturn * kMinimizer;
    return valueToReturn;
  }

  public void lockOnTarget(boolean enabled, boolean visionOn) {
    if (enabled) {
      visionIncrement += getXVal();
      double calculatedTurretAngle = (getTurretAbs() - getTurningOffset()) - visionIncrement;
      setTurret(true, calculatedTurretAngle);
    } else {
      visionIncrement = 0;
    }
  }

  public double getTurningOffset() {
    double valueToReturn;
    if (Math.abs(drive.gyro.getAngle() % 360) > 180) {
      valueToReturn = -180; // 90
    } else {
      valueToReturn = 180; // 90
    }
    return valueToReturn;
  }

  public double getTurretAbs() {
    double turningOffset = getTurningOffset();
    return (gyroOffset - (-drive.gyro.getAngle())) + turningOffset;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    x = tx.getDouble(0.0);
    y = ty.getDouble(0.0);
    targetStatus = tv.getDouble(0.0);
    targetArea = ta.getDouble(0.0);

    SmartDashboard.putNumber("Turret Motor Pozisyon", turretMotor.getSelectedSensorPosition());
    SmartDashboard.putNumber("Turret Motor Sicaklik ", turretMotor.getTemperature());
    SmartDashboard.putNumber("Turret Motor Akim ", turretMotor.getSupplyCurrent());
    SmartDashboard.putNumber("Turret Pozisyon ", getTurretPos());
  }
}