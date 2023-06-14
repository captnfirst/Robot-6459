// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
  /** Creates a new Arm. */
  double armKConv = 354.6;
  double armMinPos = -500;
  double armMaxPos = 40000;

  public WPI_TalonFX armMotor = new WPI_TalonFX(9);
  
  Timer backlashTimer = new Timer();

  public Arm() {
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

  public void resetBacklash(boolean resetStatus){
    if(resetStatus){
      backlashTimer.start();
      if (backlashTimer.get() > 0.25) {
        // Bos
      } else if(backlashTimer.get() < 0.24){
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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Arm Motor Pozisyon", armMotor.getSelectedSensorPosition());
    SmartDashboard.putNumber("Arm Motor Sicaklik ", armMotor.getTemperature());
    SmartDashboard.putNumber("Arm Motor Akim ", armMotor.getSupplyCurrent());
    SmartDashboard.putNumber("Arm Pozisyon ", getArmPos());
    //System.out.println(getArmPos() + "Arm Arm Pozisyon");
  }
}
