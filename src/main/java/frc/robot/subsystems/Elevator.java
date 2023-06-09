// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */
  double elevatorKConv = 4.85;
  double elevatorMinPos = -255000; // -117.5 // bu encoder değerini bulacağız
  double elevatorMaxPos = 0;

  // public CANSparkMax elevatorNeo = new CANSparkMax(8, MotorType.kBrushless);
  // public RelativeEncoder elevatorNeoEncoder;
  // public SparkMaxPIDController elevatorNeoPID = elevatorNeo.getPIDController();
  public WPI_TalonFX elevatorMotor = new WPI_TalonFX(8);

  public Elevator() {
    elevatorMotor.configFactoryDefault();
    elevatorMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
    elevatorMotor.setInverted(true);
    elevatorMotor.configNeutralDeadband(0.001, 0);
    elevatorMotor.setSelectedSensorPosition(0);
    elevatorMotor.configAllowableClosedloopError(0, 50, 1);
    elevatorMotor.config_kP(0, 0.1);
    elevatorMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, 0);
    elevatorMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, 0);
    elevatorMotor.configNominalOutputForward(0, 0);
    elevatorMotor.configNominalOutputReverse(0, 0);
    elevatorMotor.configPeakOutputForward(1, 0);
    elevatorMotor.configPeakOutputReverse(-1, 0);
    elevatorMotor.configMotionCruiseVelocity(400000, 0);
    elevatorMotor.configMotionAcceleration(60000, 0);
    elevatorMotor.setNeutralMode(NeutralMode.Brake);

    // elevatorNeo.restoreFactoryDefaults();
    // elevatorNeo.setIdleMode(IdleMode.kBrake);
    // elevatorNeoEncoder = elevatorNeo.getEncoder();
    // elevatorNeoEncoder.setPosition(0);
    // elevatorNeoPID.setP(0.0001);
    // elevatorNeoPID.setOutputRange(-1, 1);
    // elevatorNeoPID.setSmartMotionMaxVelocity(100000, 0);
    // elevatorNeoPID.setSmartMotionMaxAccel(100000, 0);
    // elevatorNeoPID.setSmartMotionAllowedClosedLoopError(1, 0);
  }

  public double getElevatorPos() {
    double elevatorPos = (elevatorMotor.getSelectedSensorPosition() + 0.001) / elevatorKConv;
    return elevatorPos;
  }

  public void setElevator(boolean enabled, double elevatorCm) {
    double calculatedDistance = elevatorKConv * elevatorCm;
    if (enabled) {
      elevatorMotor.set(ControlMode.MotionMagic, MathUtil.clamp(calculatedDistance, elevatorMinPos, elevatorMaxPos));
    } else {
      elevatorMotor.stopMotor();
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Elevator Motor Pozisyon", elevatorMotor.getSelectedSensorPosition());
    SmartDashboard.putNumber("Elevator Motor Sicaklik ", elevatorMotor.getTemperature());
    SmartDashboard.putNumber("Elevator Motor Akim ", elevatorMotor.getSupplyCurrent());
    SmartDashboard.putNumber("Elevator Pozisyon", getElevatorPos());
    System.out.println(getElevatorPos() + "Elevator Pozisyon");
  }
}
