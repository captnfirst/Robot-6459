// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  public WPI_TalonFX intakeMotor = new WPI_TalonFX(11);
  public DoubleSolenoid intakeSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 2, 3);
  Compressor c = new Compressor(PneumaticsModuleType.CTREPCM);
  
  public Intake() {
    intakeMotor.configFactoryDefault();
    c.enableDigital();
  }

  public void setIntakeSolenoid(DoubleSolenoid.Value solenoidStatus) {
    intakeSolenoid.set(solenoidStatus);
  }

  public void setIntake(double speed){
    intakeMotor.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
