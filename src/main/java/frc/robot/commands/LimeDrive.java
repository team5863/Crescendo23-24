// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.DriveTrain;

public class LimeDrive extends Command {
  private final DriveTrain m_driveTrain;
  DoubleSupplier speed;
  DoubleSupplier rotation;

  public LimeDrive(DriveTrain m_driveTrain, DoubleSupplier speed, DoubleSupplier rotation) {
    this.speed = speed;
    this.rotation = rotation;
    this.m_driveTrain = m_driveTrain;
    addRequirements(m_driveTrain);
  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {
    double kp = -0.02;
    double xSpeed = speed.getAsDouble();
    double zRotation = rotation.getAsDouble();
      
  if(LimelightHelpers.getTV("limelight") == true){
    zRotation = LimelightHelpers.getTX("limelight") * kp;
  }else{
    zRotation = rotation.getAsDouble();
  }

   if(LimelightHelpers.getTA("limelight") < 0.9 && LimelightHelpers.getTA("limelight") != 0) {
      xSpeed = -0.5;
    }else {
      xSpeed = speed.getAsDouble();
    }

    m_driveTrain.arcadeDrive(xSpeed, zRotation);
}
   
  @Override
  public void end(boolean interrupted) {
    m_driveTrain.arcadeDrive(0.0, 0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}