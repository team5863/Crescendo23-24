// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class ShooterSub extends SubsystemBase {

  public static CANSparkMax topNEOMotor;
  public static CANSparkMax bottomNEOMotor;

  /** Creates a new ShooterSub. */
  public ShooterSub() {
    topNEOMotor = new CANSparkMax(Constants.Drivetrain.topNEOMotoriD, MotorType.kBrushless);
    bottomNEOMotor = new CANSparkMax(Constants.Drivetrain.bottomNEOMotoriD, MotorType.kBrushless);
    
  }

  public void set(double speed){
    topNEOMotor.set(speed);
    bottomNEOMotor.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
