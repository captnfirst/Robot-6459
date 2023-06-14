// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
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
  private final DifferentialDriveOdometry m_odometry;

  /*
   *
   * Calibrated with Sysid navx but we are using ADXRS450 Gyro. This is because
   * the Navx Gyro makes our robot deviate 8-10 degrees.
   * We couldn't figure out why, so we used a different gyro.
   * 
  */
  public AHRS gyro = new AHRS(SPI.Port.kMXP); // We use for turret
  private final Gyro m_gyro = new ADXRS450_Gyro(); // We use it for driving

  public Drive() {
    m_odometry = new DifferentialDriveOdometry(
        m_gyro.getRotation2d(),
        encoderTicksToMeters(leftMaster.getSelectedSensorPosition()),
        encoderTicksToMeters(rightMaster.getSelectedSensorPosition()));

    leftMaster.configFactoryDefault();
    leftSlave0.configFactoryDefault();
    leftSlave1.configFactoryDefault();
    rightMaster.configFactoryDefault();
    rightSlave0.configFactoryDefault();
    rightSlave1.configFactoryDefault();

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

    resetEncoders();
    // Navx
    gyro.reset();
    gyro.calibrate();
  }

  public double encoderTicksToMeters(double currentEncoderValue) {
    double motorRotations = (double) currentEncoderValue / 2048;
    double wheelRotations = motorRotations / 9.36;
    double positionMeters = wheelRotations * Units.inchesToMeters(Math.PI * 5);
    return positionMeters;
  }

  public double getLeftEncoder() {
    return leftMaster.getSelectedSensorPosition();
  }

  public double getRightEncoder() {
    return rightMaster.getSelectedSensorPosition();
  }

  public void resetEncoders() {
    leftMaster.setSelectedSensorPosition(0);
    rightMaster.setSelectedSensorPosition(0);
  }

  public double getLeftEncoderVelocity() {
    return encoderTicksToMeters(leftMaster.getSelectedSensorVelocity()) * 10;
  }

  public double getRightEncoderVelocity() {
    return encoderTicksToMeters(rightMaster.getSelectedSensorVelocity()) * 10;
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(getLeftEncoderVelocity(), getRightEncoderVelocity());
  }

  public double getAverageEncoderDistance() {
    return (encoderTicksToMeters(leftMaster.getSelectedSensorPosition())
        + encoderTicksToMeters(rightMaster.getSelectedSensorPosition())) / 2.0;
  }

  public void arcadeDrive(double fwd, double rot) {
    drive.arcadeDrive(fwd, rot);
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftMaster.setVoltage(leftVolts);
    rightMaster.setVoltage(rightVolts);
    drive.feed();
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public void setMaxOutput(double maxOutput) {
    drive.setMaxOutput(maxOutput);
  }

  public void zeroHeading() {
    m_gyro.reset();
  }

  public double getTurnRate() {
    return -m_gyro.getRate();
  }

  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(
        m_gyro.getRotation2d(), encoderTicksToMeters(leftMaster.getSelectedSensorPosition()),
        encoderTicksToMeters(rightMaster.getSelectedSensorPosition()), pose);
  }

  @Override
  public void periodic() {
    m_odometry.update(
        m_gyro.getRotation2d(),
        encoderTicksToMeters(leftMaster.getSelectedSensorPosition()),
        encoderTicksToMeters(rightMaster.getSelectedSensorPosition()));

    SmartDashboard.putNumber("Battery Voltage", RobotController.getBatteryVoltage());

    SmartDashboard.putNumber("Gyro Pos", 0 + m_gyro.getAngle());

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
