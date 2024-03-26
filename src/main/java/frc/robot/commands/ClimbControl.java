// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbArm;

public class ClimbControl extends Command {
  /** Creates a new ClimbControl. */
  private final ClimbArm m_ClimbArm;
  private double speed;

  public ClimbControl(ClimbArm m_ClimbArm, double speed) {
    addRequirements(m_ClimbArm);

    this.m_ClimbArm = m_ClimbArm;
    this.speed = speed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_ClimbArm.set(speed);

    if(ClimbArm.rotateLimit.get() == false){
      m_ClimbArm.set(speed);
  }
    if(ClimbArm.rotateLimit.get() == true){
      m_ClimbArm.set(-0.8);
  }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_ClimbArm.set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
