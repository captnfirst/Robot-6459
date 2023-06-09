// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drive extends SubsystemBase {
  /** Creates a new Drive. */
  public WPI_TalonFX leftMaster = new WPI_TalonFX(5);
  public WPI_TalonFX leftSlave0 = new WPI_TalonFX(4);
  public WPI_TalonFX leftSlave1 = new WPI_TalonFX(6);
  public WPI_TalonFX rightMaster = new WPI_TalonFX(2);
  public WPI_TalonFX rightSlave0 = new WPI_TalonFX(1);
  public WPI_TalonFX rightSlave1 = new WPI_TalonFX(3);

  public DifferentialDrive drive = new DifferentialDrive(leftMaster, rightMaster);
  public AHRS gyro = new AHRS(SPI.Port.kMXP, (byte) 200);

  // Drive parameters
  // pi * wheel diameter * (pulley ratios) / (counts per rev * gearbox reduction)
  public static final double INCHES_TO_METER_CONVERSION_FACTOR = 0.0254;
  public static final double DISTANCE_PER_PULSE_IN_INCHES = 3.14 * 5.0 * 1.0 / (2048.0 * 9.36); // corrected
  public static final double DISTANCE_PER_PULSE_IN_METERS = DISTANCE_PER_PULSE_IN_INCHES * INCHES_TO_METER_CONVERSION_FACTOR;
  public static final double DISTANCE_PER_ROTATION_IN_METERS = DISTANCE_PER_PULSE_IN_METERS * 2048;

  public Drive() {
    leftMaster.configFactoryDefault();
    leftSlave0.configFactoryDefault();
    leftSlave1.configFactoryDefault();
    rightMaster.configFactoryDefault();
    rightSlave0.configFactoryDefault();
    rightSlave1.configFactoryDefault();

    gyro.reset();
    gyro.calibrate();

    leftSlave0.follow(leftMaster);
    leftSlave1.follow(leftMaster);
    rightSlave0.follow(rightMaster);
    rightSlave1.follow(rightMaster);

    leftMaster.setInverted(true);
    leftSlave0.setInverted(true);
    leftSlave1.setInverted(true);

    leftMaster.configOpenloopRamp(0.375);
    rightMaster.configOpenloopRamp(0.375);

    leftMaster.setNeutralMode(NeutralMode.Brake);
    leftSlave0.setNeutralMode(NeutralMode.Coast);
    leftSlave1.setNeutralMode(NeutralMode.Coast);
    rightMaster.setNeutralMode(NeutralMode.Brake);
    rightSlave0.setNeutralMode(NeutralMode.Coast);
    rightSlave1.setNeutralMode(NeutralMode.Coast);
  }

  double convertTicksToMeters(double ticks) {
    return ticks * DISTANCE_PER_PULSE_IN_METERS;
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(convertTicksToMeters(leftMaster.getSelectedSensorVelocity() * 10), convertTicksToMeters(rightMaster.getSelectedSensorVelocity() * 10));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Battery Voltage", RobotController.getBatteryVoltage());

    SmartDashboard.putNumber("Gyro Pos", 0 + gyro.getAngle());

    int matchnumber = DriverStation.getMatchNumber();
    DriverStation.MatchType MatchType = DriverStation.getMatchType();
    SmartDashboard.putString("matchInfo", "" + MatchType + '_' + matchnumber);

    DifferentialDriveWheelSpeeds speeds = getWheelSpeeds();
    SmartDashboard.putNumber("Drive Left Speed m per s", speeds.leftMetersPerSecond);
    SmartDashboard.putNumber("Drive Right Speed m per s", speeds.rightMetersPerSecond);
  }

  static double map(double x, double in_min, double in_max, double out_min, double out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  }
}
