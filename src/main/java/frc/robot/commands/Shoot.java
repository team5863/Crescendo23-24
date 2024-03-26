package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class Shoot extends Command {
    private final Shooter m_shooter;

    Double speed;

    public Shoot(Shooter m_shooter, Double speed){
        this.m_shooter = m_shooter;
        this.speed = speed;
        addRequirements(m_shooter);
    }

    @Override
    public void execute() {
        m_shooter.shoot(speed);
    }

    @Override
    public void end(boolean interrupted) {
        m_shooter.shoot(0);
    }
}
