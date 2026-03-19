package frc.robot.subsystems;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

import frc.robot.Constantes.ConstantesShooter;
import frc.robot.Hardwares.HardwaresShooter;
import frc.robot.StatesMachines.StateMachineShooter;
import frc.robot.StatesMachines.StateMachineShooter.Estado;

@SuppressWarnings("unused")
public class Shooter extends SubsystemBase {

    private final HardwaresShooter io = new HardwaresShooter();
    private final StateMachineShooter sm = new StateMachineShooter();

    private double rpmAlvo = ConstantesShooter.Velocidade.NORMAL.rpm;

    private final LinearFilter filtroBateria =
    LinearFilter.singlePoleIIR(0.1, 0.02);

    private boolean alimentando = false;

    private final double[] DUTIES = { 0.30, 0.40, 0.50 };
    private int dutyAtual = 0;

    private final LinearFilter filtroDashboard =
        LinearFilter.singlePoleIIR(ConstantesShooter.FILTRO_RPM_TAU_S, 0.02);

    private double somaKV = 0.0;
    private int amostrasKV = 0;

    private double rpmFiltradoDashboard = 0.0;

    private boolean modoTesteFF = false;

    private ConstantesShooter.Velocidade velocidade =
        ConstantesShooter.Velocidade.NORMAL;

    private double entrouNaFaixaEm = -1.0;

    public void setVelocidade(ConstantesShooter.Velocidade vel) {
        velocidade = vel;
        rpmAlvo = vel.rpm;
    }

    public void setAlimentando(boolean valor) {
        alimentando = valor && sm.get() != Estado.PARADO;
    }

    public void setRpmDireto(double rpm) {
        rpmAlvo = rpm;
    }

    public ConstantesShooter.Velocidade getVelocidade() {
    return velocidade;
    }

    public void atirarFrente() {
        sm.set(StateMachineShooter.Estado.ATIRANDO_FRENTE);
    }

    public void atirarTras() {
        sm.set(StateMachineShooter.Estado.ATIRANDO_TRAS);
    }

    public void parar() {
        sm.set(StateMachineShooter.Estado.PARADO);
    }

    public boolean pronto() {
        return Math.abs(rpmAlvo - rpmFiltradoDashboard)
            < ConstantesShooter.TOLERANCIA_RPM;
    }

    public void testandoTiro() {
        modoTesteFF = true;
    }

    public void pararTesteFF() {
        modoTesteFF = false;
        io.arlindo.stopMotor();
    }
    public void proximoDuty() {
        dutyAtual = (dutyAtual + 1)% DUTIES.length;
        somaKV = 0.0;
        amostrasKV = 0;
    }
    public boolean prontoEstavel() {

        double rpm = Math.abs(io.arlindoEncoder.getVelocity());
        double erro = Math.abs(rpmAlvo - rpm);
        double agora = Timer.getFPGATimestamp();

        if (erro < ConstantesShooter.TOLERANCIA_RPM) {
            if (entrouNaFaixaEm < 0.0) {
                entrouNaFaixaEm = agora;
            }
        } else if (erro > ConstantesShooter.TOLERANCIA_RPM_SAIDA) {
            entrouNaFaixaEm = -1.0;
            return false;
        }

        return entrouNaFaixaEm > 0.0 &&
               (agora - entrouNaFaixaEm)
                   >= ConstantesShooter.TEMPO_ESTABILIZACAO_S;
    }

    private void publicarTelemetria() {
        SmartDashboard.putNumber("Shooter/RPM", rpmFiltradoDashboard);
        SmartDashboard.putNumber("Shooter/RPM Alvo", rpmAlvo);
        SmartDashboard.putNumber("Teste/Duty", DUTIES[dutyAtual]);
        SmartDashboard.putNumber("Teste/RPM", io.arlindoEncoder.getVelocity());
        SmartDashboard.putNumber("Teste/Bateria", RobotController.getBatteryVoltage());
    }

    private final SimpleMotorFeedforward feedforward = 
        new SimpleMotorFeedforward(
        ConstantesShooter.kS, 
        ConstantesShooter.kV
        );

    @Override
public void periodic() {

    double rpmBruto = Math.abs(io.arlindoEncoder.getVelocity());
    rpmFiltradoDashboard = filtroDashboard.calculate(rpmBruto);

    if (modoTesteFF) {
        io.arlindo.stopMotor();
        io.arlindo.set(DUTIES[dutyAtual]);
        calcularKVAutomatico();
        publicarTelemetria();
        return;
    }


    switch (sm.get()) {

        case ATIRANDO_FRENTE -> aplicarControleVelocidade(rpmAlvo);

        case ATIRANDO_TRAS -> aplicarControleVelocidade(-rpmAlvo);

        case PARADO -> {
            io.arlindo.stopMotor();
            entrouNaFaixaEm = -1.0;
        }
    }
    publicarTelemetria();

     SmartDashboard.putNumber("Shooter/RPM", rpmFiltradoDashboard);
        SmartDashboard.putNumber("Shooter/RPM Alvo", rpmAlvo);
}



    private void aplicarControleVelocidade(double alvoRpm) {

        double alvoCompensado = alvoRpm;

        if (alimentando) {
            alvoCompensado += Math.copySign(
                ConstantesShooter.RPM_ANTI_DROP,
                alvoRpm
            );
        }

        double rps = Math.abs(alvoCompensado) / 60;

        double bateria = filtroBateria.calculate(
            RobotController.getBatteryVoltage()
        );
        double ffVolts = feedforward.calculate(rps);
        double ffDutyCycle = Math.copySign(ffVolts / bateria, alvoCompensado);

            io.arlindopid.setSetpoint(
                alvoCompensado,
                ControlType.kVelocity,
                ClosedLoopSlot.kSlot0,
                ffDutyCycle
            );
    }
    private void calcularKVAutomatico() {

    double rpm = Math.abs(io.arlindoEncoder.getVelocity());
    if (rpm < 300) return; // evita região de kS / ruído

    double duty = DUTIES[dutyAtual];
    double bateria = RobotController.getBatteryVoltage();

    double volts = duty * bateria;
    double rps = rpm / 60.0;

    double kVinst = volts / rps;

    somaKV += kVinst;
    amostrasKV++;

    double kVmedio = somaKV / amostrasKV;

    SmartDashboard.putNumber("FF/kV Instantaneo", kVinst);
    SmartDashboard.putNumber("FF/kV Medio", kVmedio);
    SmartDashboard.putNumber("FF/Amostras", amostrasKV);
}
}