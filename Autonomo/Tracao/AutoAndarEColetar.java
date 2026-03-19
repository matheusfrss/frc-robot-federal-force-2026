package frc.robot.commands.Autonomo.Tracao;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;

import frc.robot.subsystems.Traction;
import frc.robot.Extras.AngulosPresetPivot;
import frc.robot.commands.Autonomo.intake.Autobaixointake;
import frc.robot.commands.Pivot.MoverPivotPreset;
import frc.robot.subsystems.IntakeFloor;

public class AutoAndarEColetar extends ParallelDeadlineGroup {

    public AutoAndarEColetar(Traction traction, IntakeFloor intake) {

        super(

            // DEADLINE 
            new AndarEncoder(traction, -0.5, 0.7),

            // Intake + pivot juntos
            new StartEndCommand(
                () -> {
                    intake.forcarHold();
                    intake.IntakeOn();
                },
                () -> {
                    intake.PararIntake();
                },
                intake
            )

        );
    }
}
