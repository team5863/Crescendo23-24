package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;

public class TeleOPDrive extends Command {
  private final DriveTrain m_driveTrain;
  DoubleSupplier speed;
  DoubleSupplier rotation;
 
  public TeleOPDrive(DriveTrain m_driveTrain, DoubleSupplier speed, DoubleSupplier rotation) {
   this.m_driveTrain = m_driveTrain;
   this.speed = speed;
   this.rotation = rotation;
   addRequirements(m_driveTrain);
  }

 
  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_driveTrain.arcadeDrive(speed.getAsDouble(), rotation.getAsDouble());
  }

 
  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
