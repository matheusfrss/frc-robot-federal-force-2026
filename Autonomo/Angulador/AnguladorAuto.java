package frc.robot.commands.Autonomo.Angulador;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Angulador;
import frc.robot.Extras.AnguloPreset;
import frc.robot.StatesMachines.StateMachineAngulador;

public class AnguladorAuto extends Command {

    private final Angulador angulador;
    private final AnguloPreset preset;

    public AnguladorAuto(Angulador angulador, AnguloPreset preset) {
        this.angulador = angulador;
        this.preset = preset;
        addRequirements(angulador);
    }

    @Override
    public void initialize() {
        angulador.moverParaPreset(preset);
    }

    @Override
    public boolean isFinished() {
        return angulador.getEstado() 
            == StateMachineAngulador.Estado.HOLD;
    }
}


//new AnguladorAuto(angulador, AnguloPreset.AUTO)