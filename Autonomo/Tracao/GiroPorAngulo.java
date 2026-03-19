package frc.robot.commands.Autonomo.Tracao;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Traction;

public class GiroPorAngulo extends Command {

    private final Traction traction;
    private final double anguloAlvo;
    private final double velocidade = 0.7;

    private double anguloInicial;

    public GiroPorAngulo(Traction traction, double anguloAlvo) {
        this.traction = traction;
        this.anguloAlvo = anguloAlvo;
        addRequirements(traction);
    }

    @Override
    public void initialize() {
        anguloInicial = traction.getYaw(); // salva o ângulo atual
    }

    @Override
    public void execute() {
        // gira pro lado certo dependendo do sinal do anguloAlvo
        double direcao = Math.signum(anguloAlvo);
        traction.arcadeMode(0, -velocidade * direcao);
    }

    @Override
    public void end(boolean interrupted) {
        traction.stop();
    }

    @Override
    public boolean isFinished() {
        double anguloAtual = traction.getYaw();

        // para quando atingir o anguloAlvo (em graus)
        return Math.abs(anguloAtual - anguloInicial) >= Math.abs(anguloAlvo);
    }
}
