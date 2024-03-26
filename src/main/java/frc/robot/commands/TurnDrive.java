// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TurnDrive extends Command {
  /** Creates a new TurnDrive. */
  private final DriveTrain m_driveTrain;

  private double heading;
  public TurnDrive(DriveTrain m_driveTrain, double heading) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_driveTrain = m_driveTrain;
    addRequirements(m_driveTrain);
    //DriveTrain.gyro.setYaw(0);
    this.heading = heading;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putNumber("TargetHeading", heading);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_driveTrain.turn(heading);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //DriveTrain.gyro.setYaw(0);
    
    m_driveTrain.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return DriveTrain.kPID_headingRight.atSetpoint();
  }
}
