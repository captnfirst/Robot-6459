// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Head extends SubsystemBase {
  /** Creates a new Head. */
  public WPI_TalonFX headMotor = new WPI_TalonFX(10);

  public Head() {
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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Head Motor Pozisyon", headMotor.getSelectedSensorPosition());
    SmartDashboard.putNumber("Head Motor Sicaklik ", headMotor.getTemperature());
    SmartDashboard.putNumber("Head Motor Akim ", headMotor.getSupplyCurrent());
    SmartDashboard.putNumber("Head Pozisyon", getHeadPos());
    System.out.println(getHeadPos() + "Head Pozisyon");
  }
}
