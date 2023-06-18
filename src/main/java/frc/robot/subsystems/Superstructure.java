// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Superstructure extends SubsystemBase {
  /** Creates a new Superstructure. */
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
  public final Gyro m_gyro = new ADXRS450_Gyro(); // We use it for driving

  double armKConv = 354.6;
  double armMinPos = -500;
  double armMaxPos = 40000;

  public WPI_TalonFX armMotor = new WPI_TalonFX(9);

  Timer backlashTimer = new Timer();

  public WPI_TalonFX headMotor = new WPI_TalonFX(10);

  public WPI_TalonFX intakeMotor = new WPI_TalonFX(11);
  public DoubleSolenoid intakeSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 2, 3);
  Compressor c = new Compressor(PneumaticsModuleType.CTREPCM);

  public double turretKConv = -1055.55;
  public double turretMinPos = -548000; // 240000
  public double turretMaxPos = 548000; // -240000

  public WPI_TalonFX turretMotor = new WPI_TalonFX(7);

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
  
  public Superstructure() {
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

    intakeMotor.configFactoryDefault();
    c.enableDigital();

    headMotor.configFactoryDefault();
    headMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
    headMotor.configNeutralDeadband(0.001, 0);
    headMotor.setSelectedSensorPosition(0);
    headMotor.configAllowableClosedloopError(0, 50, 1);
    headMotor.config_kP(0, 0.1);
    headMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, 0);
    headMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, 0);
    headMotor.configNominalOutputForward(0, 0);
    headMotor.configNominalOutputReverse(0, 0);
    headMotor.configPeakOutputForward(0.45, 0);
    headMotor.configPeakOutputReverse(-0.45, 0);
    headMotor.configMotionCruiseVelocity(8000, 0);
    headMotor.configMotionAcceleration(24000, 0);

    armMotor.configFactoryDefault();
    armMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
    armMotor.configNeutralDeadband(0.001, 0);
    armMotor.setSelectedSensorPosition(0);
    armMotor.configAllowableClosedloopError(0, 50, 1);
    armMotor.config_kP(0, 0.5); // 0.3
    armMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, 0);
    armMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, 0);
    armMotor.configNominalOutputForward(0, 0);
    armMotor.configNominalOutputReverse(0, 0);
    armMotor.configPeakOutputForward(0.45, 0);
    armMotor.configPeakOutputReverse(-0.45, 0);
    armMotor.configMotionCruiseVelocity(15000, 0);
    armMotor.configMotionAcceleration(2500, 0);

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
    if (Math.abs(gyro.getAngle() % 360) > 180) {
      valueToReturn = -180; // 90
    } else {
      valueToReturn = 180; // 90
    }
    return valueToReturn;
  }

  public double getTurretAbs() {
    double turningOffset = getTurningOffset();
    return (gyroOffset - (-gyro.getAngle())) + turningOffset;
  }

  public void setIntakeSolenoid(DoubleSolenoid.Value solenoidStatus) {
    intakeSolenoid.set(solenoidStatus);
  }

  public void setIntake(double speed) {
    intakeMotor.set(speed);
  }

  public double getHeadPos() {
    double headPos = -headMotor.getSelectedSensorPosition();
    return headPos;
  }

  public void setHead(boolean enabled, double rawHeadPos) {
    if (enabled) {
      headMotor.set(ControlMode.MotionMagic, -rawHeadPos);
    } else {
      headMotor.stopMotor();
    }
  }

  public double getArmPos() {
    double armPos = (armMotor.getSelectedSensorPosition() + 0.001) / armKConv;
    return armPos;
  }

  public void setArm(boolean enabled, double armDegrees) {
    double calculatedAngle = armKConv * armDegrees;
    if (enabled) {
      armMotor.set(ControlMode.MotionMagic, MathUtil.clamp(calculatedAngle, armMinPos, armMaxPos));
    } else {
      armMotor.stopMotor();
    }
  }

  public void resetBacklash(boolean resetStatus) {
    if (resetStatus) {
      backlashTimer.start();
      if (backlashTimer.get() > 0.25) {
        // Bos
      } else if (backlashTimer.get() < 0.24) {
        armMotor.set(-0.05);
        // headMotor.set(-0.05);
      } else {
        armMotor.set(0);
        armMotor.setSelectedSensorPosition(0);
        // headMotor.set(0);
        // headMotor.setSelectedSensorPosition(0);
      }
    }
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
    // This method will be called once per scheduler run
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

    SmartDashboard.putNumber("Arm Motor Pozisyon", armMotor.getSelectedSensorPosition());
    SmartDashboard.putNumber("Arm Motor Sicaklik ", armMotor.getTemperature());
    SmartDashboard.putNumber("Arm Motor Akim ", armMotor.getSupplyCurrent());
    SmartDashboard.putNumber("Arm Pozisyon ", getArmPos());

    SmartDashboard.putNumber("Head Motor Pozisyon", headMotor.getSelectedSensorPosition());
    SmartDashboard.putNumber("Head Motor Sicaklik ", headMotor.getTemperature());
    SmartDashboard.putNumber("Head Motor Akim ", headMotor.getSupplyCurrent());
    SmartDashboard.putNumber("Head Pozisyon", getHeadPos());

    x = tx.getDouble(0.0);
    y = ty.getDouble(0.0);
    targetStatus = tv.getDouble(0.0);
    targetArea = ta.getDouble(0.0);

    SmartDashboard.putNumber("Turret Motor Pozisyon", turretMotor.getSelectedSensorPosition());
    SmartDashboard.putNumber("Turret Motor Sicaklik ", turretMotor.getTemperature());
    SmartDashboard.putNumber("Turret Motor Akim ", turretMotor.getSupplyCurrent());
    SmartDashboard.putNumber("Turret Pozisyon ", getTurretPos());
  }

  static double map(double x, double in_min, double in_max, double out_min, double out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  }
}
