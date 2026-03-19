package frc.robot.commands.Autonomo.intake;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Extras.AngulosPresetPivot;
import frc.robot.commands.Pivot.MoverPivotPreset;
import frc.robot.subsystems.IntakeFloor;

@SuppressWarnings ("unused")
public class Autobaixointake extends SequentialCommandGroup {

    public Autobaixointake(IntakeFloor intake) {

        new StartEndCommand(
                () -> {
                    intake.moverParaPreset(AngulosPresetPivot.ALTO); 
                    intake.forcarHold();
                },
                () -> {
                    intake.PararIntake();
                },
                intake
            );
        
    }
;}