package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constantes.ConstantesClimber;
import frc.robot.Hardwares.HardwaresClimber;
import frc.robot.Kinematics.KinematicsClimber;
import frc.robot.StatesMachines.StateMachineClimber;

public class Climber extends SubsystemBase {

    private final HardwaresClimber io = new HardwaresClimber();
    private final StateMachineClimber sm = new StateMachineClimber();

    private double alvoAltura = Double.NaN;
    private double rotacoesHold = 0.0;
    private double manualOutput = 0.0;
    private double ultimoSetpointRotacoes = Double.NaN;

    @SuppressWarnings("unused")
    private double posicaoAnterior = 0.0;
    private double tempoSemMovimento = 0.0;
    private double ultimoTimestamp = Timer.getFPGATimestamp();

    public double getAltura() {
        return KinematicsClimber.rotacoesMotorParaAltura(
            io.encoder.getPosition()
        );
    }

    public StateMachineClimber.Estado getEstado() {
        return sm.get();
    }

    public void zerarEncoder() {
        io.encoder.setPosition(0.0);
    }

    public void irParaAltura(double alturaMetros) {

        alturaMetros = MathUtil.clamp(
            alturaMetros,
            ConstantesClimber.LIMITE_INFERIOR,
            ConstantesClimber.LIMITE_SUPERIOR
        );

        alvoAltura = alturaMetros;

        definiSetpoint(KinematicsClimber.alturaParaRotacoesMotor(alturaMetros));

        sm.set(StateMachineClimber.Estado.MOVENDO);
    }

    public void controleManual(double output) {
        manualOutput = MathUtil.clamp(output, -0.4, 0.4);
        sm.set(StateMachineClimber.Estado.MANUAL);
    }

    public void  resetarFalha(){
        tempoSemMovimento = 0.0;
        alvoAltura = Double.NaN;
        sm.set(StateMachineClimber.Estado.IDLE);
        io.motor.stopMotor();
    }


    private void executarMovendo() {

        double alturaAtual = getAltura();
        double erro = alvoAltura - alturaAtual;

        if (Math.abs(erro) <= ConstantesClimber.MARGEM_ERRO) {
            rotacoesHold = io.encoder.getPosition();
            sm.set(StateMachineClimber.Estado.HOLD);
            return;
        }

        definiSetpoint(alvoAltura);
    }

    private void executarHold() {
        definiSetpoint(rotacoesHold);
    }

    private void definiSetpoint(double rotacoes){
        if (Double.isNaN(ultimoSetpointRotacoes) || Math.abs(ultimoSetpointRotacoes - rotacoes) > 1e-4){
            io.pid.setSetpoint(
                rotacoes, SparkBase.ControlType.kPosition
            );
            ultimoSetpointRotacoes = rotacoes;
        }
    }


    @Override
    public void periodic() {

        double agora = Timer.getFPGATimestamp();
        double dt = agora - ultimoTimestamp;
        if(dt <= 0.0) {
            dt = ConstantesClimber.DT;
        }
        double posicaoAtual = io.encoder.getPosition();
        double velocidade =
           io.encoder.getVelocity();
        double erroAltura =
            Double.isNaN(alvoAltura)
                ? 0.0
                : Math.abs(alvoAltura - getAltura());

        if (sm.is(StateMachineClimber.Estado.MOVENDO)
            && erroAltura > ConstantesClimber.ERRO_TRAVAMENTO) {

            if (Math.abs(velocidade)
                < ConstantesClimber.VELOCIDADE_MIN) {

                tempoSemMovimento += dt;
            } else {
                tempoSemMovimento = 0.0;
            }
        } else {
            tempoSemMovimento = 0.0;
        }

        if (tempoSemMovimento
            >= ConstantesClimber.TEMPO_MAX_TRAVADO) {

            sm.set(StateMachineClimber.Estado.FALHA);
            io.motor.stopMotor();
        }

        posicaoAnterior = posicaoAtual;
        ultimoTimestamp = agora;

        switch (sm.get()) {
            case MOVENDO -> executarMovendo();
            case HOLD    -> executarHold();
            case MANUAL  -> {
                double altura = getAltura();
                double outputSeguro = manualOutput;
                if ((altura <= ConstantesClimber.LIMITE_INFERIOR
                        && outputSeguro < 0.0)
                    || (altura >= ConstantesClimber.LIMITE_SUPERIOR
                        && outputSeguro > 0.0)) {
                    outputSeguro = 0.0;
                }
                io.motor.set(outputSeguro);
            }
            default      -> io.motor.stopMotor();
        }

        SmartDashboard.putNumber("Climber/Altura", getAltura());
        SmartDashboard.putString("Climber/Estado", sm.get().name());
    }
}
