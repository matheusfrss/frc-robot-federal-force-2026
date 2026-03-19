package frc.robot.commands.Autonomo.Shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.subsystems.*;

public class AutoShootCompleto extends Command {

    private final Traction traction;
    private final Limelight limelight;
    private final Shooter shooter;
    private final Angulador angulador;
    private final Index boquinha;
    private final Index index;

    private static final double KP_ROT = 0.045;
    private static final double ERRO_TX_OK = 0.5;
    private static final double TEMPO_ESTAVEL = 0.3;

    private static final double TEMPO_BOQUINHA = 0.4;
    private static final double TEMPO_INDEX = 0.9;
    private static final double TEMPO_TOTAL = 2.5;

    private double tempoDentro = 0;
    private double inicioDisparo = -1;

    public AutoShootCompleto(
        Traction traction,
        Limelight limelight,
        Shooter shooter,
        Angulador angulador,
        Index boquinha,
        Index index
    ) {
        this.traction = traction;
        this.limelight = limelight;
        this.shooter = shooter;
        this.angulador = angulador;
        this.boquinha = boquinha;
        this.index = index;

        addRequirements(traction, shooter, angulador, boquinha, index);
    }

    @Override
    public void initialize() {
        tempoDentro = 0;
        inicioDisparo = -1;
    }

    @Override
    public void execute() {

        // ============================
        // 1️⃣ Se não tem alvo, não faz nada
        // ============================
        if (!limelight.temAlvo()) {
            traction.stop();
            shooter.parar();
            return;
        }

        // ============================
        // 2️⃣ ALINHAMENTO HORIZONTAL
        // ============================
        double erroX = limelight.getTxFiltrado();
        double rot = erroX * KP_ROT;
        rot = MathUtil.clamp(rot, -0.5, 0.5);

        if (Math.abs(erroX) < ERRO_TX_OK) {
            rot = 0;
        }

        traction.arcadeMode(0.0, rot);

        // ============================
        // 3️⃣ DISTÂNCIA
        // ============================
        double distancia = limelight.getDistanciaFiltrada();

        // ============================
        // 4️⃣ CALCULA RPM ANTES DE LIGAR
        // ============================
        double rpm = 2100 + (distancia * 180);
        shooter.setRpmDireto(rpm);

        // Agora sim liga o shooter se estiver parado
        shooter.atirarFrente();

        // ============================
        // 5️⃣ ÂNGULO DINÂMICO
        // ============================
        double angulo = 25 + (distancia * 2.0);
        angulador.moverParaAngulo(angulo);

        // ============================
        // 6️⃣ VERIFICA ESTABILIDADE
        // ============================
        boolean alinhado = Math.abs(erroX) < ERRO_TX_OK;
        boolean shooterOk = shooter.prontoEstavel();
        boolean anguloOk = angulador.noAngulo();

        boolean pronto = alinhado && shooterOk && anguloOk;

        if (pronto) {
            tempoDentro += 0.02;
        } else {
            tempoDentro = 0;
        }

        // ============================
        // 7️⃣ INICIA DISPARO
        // ============================
        if (tempoDentro > TEMPO_ESTAVEL && inicioDisparo < 0) {
            inicioDisparo = Timer.getFPGATimestamp();
        }

        if (inicioDisparo > 0) {

            double tempoAtual =
                Timer.getFPGATimestamp() - inicioDisparo;

            if (tempoAtual > TEMPO_BOQUINHA) {
                boquinha.ligar();
            }

            if (tempoAtual > TEMPO_INDEX) {
                index.ligar();
            }
        }

        // ============================
        // DEBUG
        // ============================
        SmartDashboard.putBoolean("Auto/Alinhado", alinhado);
        SmartDashboard.putBoolean("Auto/ShooterOK", shooterOk);
        SmartDashboard.putBoolean("Auto/AnguloOK", anguloOk);
        SmartDashboard.putNumber("Auto/RPM", rpm);
        SmartDashboard.putNumber("Auto/Distancia", distancia);
    }

    @Override
    public boolean isFinished() {

        if (inicioDisparo < 0) return false;

        return Timer.getFPGATimestamp() - inicioDisparo
                > TEMPO_TOTAL;
    }

    @Override
    public void end(boolean interrupted) {

        traction.stop();
        shooter.parar();
        boquinha.desligar();
        index.desligar();
    }
}