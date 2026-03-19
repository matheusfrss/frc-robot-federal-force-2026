package frc.robot.commands.Autonomo.intake;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import frc.robot.subsystems.IntakeFloor;
import frc.robot.commands.Pivot.MoverPivotPreset;
import frc.robot.Extras.AngulosPresetPivot;

public class AutoIntakeFloor extends SequentialCommandGroup {

    public AutoIntakeFloor(IntakeFloor intake) {

        addCommands(

     new MoverPivotPreset(intake, AngulosPresetPivot.ALTO)
     );
    }
}

