package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbArm;

public class AutoClimb extends Command {
  private final ClimbArm m_ClimbArm;
  private double position;

  public AutoClimb(ClimbArm m_ClimbArm, double position) {
    addRequirements(m_ClimbArm);

    this.m_ClimbArm = m_ClimbArm;
    this.position = position;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_ClimbArm.Positions(position);

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
