package frc.robot.commands.Autonomo.Auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Traction;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Angulador;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.IntakeFloor;

public class AutoShoot extends Command {

    private final Traction traction;
    private final Limelight limelight;
    private final Shooter shooter;
    private final Angulador angulador;
    private final Index boquinha;
    private final Index index;
    private final IntakeFloor intake;

    private static final double KP_ROT = 0.04;
    private static final double ERRO_TX_OK = 0.5;

    private static final double TEMPO_ESTAVEL = 0.25;
    private static final double ATRASO_BOQUINHA = 0.6;
    private static final double ATRASO_INDEX = 1.4;
    private static final double TEMPO_TOTAL_DISPARO = 3.5;

    private double tempoDentro = 0;
    private double inicioDisparo = -1;

    public AutoShoot(
        Traction traction,
        Limelight limelight,
        Shooter shooter,
        Angulador angulador,
        Index boquinha,
        Index index,
        IntakeFloor intake
    ) {
        this.traction = traction;
        this.limelight = limelight;
        this.shooter = shooter;
        this.angulador = angulador;
        this.boquinha = boquinha;
        this.index = index;
        this.intake = intake;

        addRequirements(traction, shooter, angulador, boquinha, index, intake);
    }

    @Override
    public void initialize() {
        shooter.atirarFrente();
        tempoDentro = 0;
        inicioDisparo = -1;
    }

    @Override
    public void execute() {

        if (!limelight.temAlvo()) {
            traction.stop();
            return;
        }

        //ALINHAMENTO HORIZONTAL
 
        double erroX = limelight.getTxFiltrado();
        double rot = erroX * KP_ROT;
        traction.arcadeMode(0.0, rot);

        // CALCULO DINAMICO RPM

        double distancia = limelight.getDistanciaFiltrada();

        double rpm = 2100 + (distancia * 180); 

        shooter.setRpmDireto(rpm);

        // ANGULO DINAMICO

        double angulo = 25 + (distancia * 2.0);
        angulador.moverParaAngulo(angulo);

        //  VERIFICAR SE ESTA PRONTO

        boolean alinhado = Math.abs(erroX) < ERRO_TX_OK;
        boolean shooterOk = shooter.prontoEstavel();
        boolean anguloOk = angulador.noAngulo();

        boolean pronto = alinhado && shooterOk && anguloOk;

        if (pronto) {
            tempoDentro += 0.02;
        } else {
            tempoDentro = 0;
        }

        // INICIAR DISPARO

        if (tempoDentro > TEMPO_ESTAVEL && inicioDisparo < 0) {
            inicioDisparo = Timer.getFPGATimestamp();
        }

        if (inicioDisparo > 0) {

            double tempoAtual = Timer.getFPGATimestamp() - inicioDisparo;

            if (tempoAtual > ATRASO_BOQUINHA) {
                boquinha.ligar();
            }

            if (tempoAtual > ATRASO_INDEX) {
                index.ligar();
                intake.IntakeReverse();
            }
        }
    }

    @Override
    public boolean isFinished() {
        if (inicioDisparo < 0) return false;

        return Timer.getFPGATimestamp() - inicioDisparo
            > TEMPO_TOTAL_DISPARO;
    }

    @Override
    public void end(boolean interrupted) {
        traction.stop();
        shooter.parar();
        boquinha.desligar();
        index.desligar();
        intake.PararIntake();
    }
}