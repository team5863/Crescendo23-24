package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.TeleOPDrive;
import frc.robot.subsystems.DriveTrain;

public class LeaveStudents extends SequentialCommandGroup{

    public LeaveStudents(DriveTrain m_driveTrain){

        addCommands(

            new ParallelDeadlineGroup(
                new WaitCommand(6),
                new TeleOPDrive(m_driveTrain, ()-> 0, ()-> 0.6)
            )
        );

    }


    
}
    