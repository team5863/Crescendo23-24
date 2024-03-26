package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Serializer;
import frc.robot.subsystems.Shooter;

public class SpeakerShoot extends Command {
    private final Shooter m_shooter;
    private final Serializer m_serialize;

    Double speed;
    Double speed2;

    public SpeakerShoot(Shooter m_shooter, Double speed, Serializer m_serialize, Double speed2){
        this.m_shooter = m_shooter;
        this.m_serialize = m_serialize;
        this.speed = speed;
        this.speed2 = speed2;
        addRequirements(m_shooter);
    }

    @Override
    public void execute() {
     
        m_shooter.shoot(speed);
        m_serialize.serialize(speed2);
        
    }

    @Override
    public void end(boolean interrupted) {
        m_shooter.shoot(0);
        m_serialize.serialize(0);
    }
}
