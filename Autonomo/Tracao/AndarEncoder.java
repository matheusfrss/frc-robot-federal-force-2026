    package frc.robot.commands.Autonomo.Tracao;

    import edu.wpi.first.wpilibj2.command.Command;
    import frc.robot.subsystems.Traction;

    public class AndarEncoder extends Command {

        
        private final Traction traction;
        private final double velocidade;
        private final double distancia;

        private double distanciaInicial;

        public AndarEncoder(Traction traction, double velocidade, double distancia) {
            this.traction = traction;
            this.velocidade = velocidade;
            this.distancia = distancia;
            addRequirements(traction);
        }

        @Override
        public void initialize() {
            // Salva a distância atual em vez de confiar no reset
            distanciaInicial = traction.getAverageDistance();
        }

        @Override
        public void execute() {
            traction.arcadeMode(-velocidade, 0);
        }

        @Override
        public void end(boolean interrupted) {
            traction.stop();
        }

        @Override
        public boolean isFinished() {
            double distanciaAtual = traction.getAverageDistance();
            return Math.abs(distanciaAtual - distanciaInicial) >= distancia;
        }
    }