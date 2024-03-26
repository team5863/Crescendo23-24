package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intaker;

public class Intake extends Command {
    private final Intaker m_intake;

    Double speed;

    public Intake(Intaker m_intake, Double speed){
        this.m_intake = m_intake;
        this.speed = speed;
        addRequirements(m_intake);
    }

    @Override
    public void execute() {
       m_intake.intake(speed);
    }

    @Override
    public void end(boolean interrupted) {
        m_intake.intake(0);
    }
}
