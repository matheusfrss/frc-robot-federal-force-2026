package frc.robot.subsystems;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constantes.ConstantesAngulador;
import frc.robot.Extras.AnguloPreset;
import frc.robot.Hardwares.HardwaresAngulador;
import frc.robot.Kinematics.KInematicsAngulador;
import frc.robot.StatesMachines.StateMachineAngulador;

public class Angulador extends SubsystemBase {

    private final HardwaresAngulador io = new HardwaresAngulador();
    private final StateMachineAngulador sm = new StateMachineAngulador();

    private final ArmFeedforward ff =
        new ArmFeedforward(
            ConstantesAngulador.FFAlinhador.kS,
            ConstantesAngulador.FFAlinhador.kG,
            ConstantesAngulador.FFAlinhador.kV
        );

    private final TrapezoidProfile.Constraints constraints =
        new TrapezoidProfile.Constraints(
            ConstantesAngulador.MAX_VEL,
            ConstantesAngulador.MAX_ACC
        );

    private TrapezoidProfile.State goal = new TrapezoidProfile.State();
    private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();

    private double alvoGraus = Double.NaN;
    private double anguloHold = 0.0;

    private double manualOutput = 0.0;
    private double testeOutput = 0.0;

    private double tempoTravado = 0.0;
    private double ultimoTimestamp = Timer.getFPGATimestamp();

    public double getAngulo() {
        return KInematicsAngulador.rotacoesParaGraus(
            io.encoder.getPosition()
        );
    }

    public boolean noAngulo() {
        return Math.abs(alvoGraus - getAngulo())
            < ConstantesAngulador.ERRO_TOLERANCIA;
    }


    public void moverParaAngulo(double graus) {

        double limitado = MathUtil.clamp(
            graus,
            ConstantesAngulador.LIMITE_INFERIOR,
            ConstantesAngulador.LIMITE_SUPERIOR
        );

        alvoGraus = limitado;
        goal = new TrapezoidProfile.State(limitado, 0.0);
        setpoint = new TrapezoidProfile.State(getAngulo(), 0.0);

        sm.set(StateMachineAngulador.Estado.PERFIL);
    }

    public void moverParaPreset(AnguloPreset preset) {
        moverParaAngulo(preset.graus);
    }

    public StateMachineAngulador.Estado getEstado(){
        return sm.get();
    }

    public void controleManual(double output) {
        manualOutput = output;
        sm.set(StateMachineAngulador.Estado.MANUAL);
    }

    public void testeMotor(double output) {
        testeOutput = output;
        sm.set(StateMachineAngulador.Estado.TESTE);
    }

    public void parar() {
        io.motor.stopMotor();
    }

    @Override
    public void periodic() {

        double agora = Timer.getFPGATimestamp();
        double dt = agora - ultimoTimestamp;
        if (dt <= 0.0) dt = ConstantesAngulador.DT;

        switch (sm.get()) {

            case PERFIL -> executarPerfil(dt);

            case HOLD -> executarHold();

            case MANUAL -> io.motor.set(manualOutput);

            case TESTE -> io.motor.set(testeOutput);

            case FALHA -> io.motor.stopMotor();

            default -> io.motor.stopMotor();
        }

        detectarTravamento(dt);

        SmartDashboard.putNumber("Angulador/Angulo", getAngulo());
        SmartDashboard.putString("Angulador/Estado", sm.get().name());

        ultimoTimestamp = agora;
    }

    private void executarPerfil(double dt) {

        TrapezoidProfile profile =
            new TrapezoidProfile(constraints);

        setpoint = profile.calculate(dt, setpoint, goal);

        double ffVolts = ff.calculate(
            Math.toRadians(getAngulo()),
            Math.toRadians(setpoint.velocity)
        );

        io.pid.setSetpoint(
            KInematicsAngulador.grausParaRotacoes(setpoint.position),
            SparkBase.ControlType.kPosition,
            ClosedLoopSlot.kSlot0,
            ffVolts
        );

        boolean chegou =
            Math.abs(goal.position - setpoint.position)
                < ConstantesAngulador.MARGEM_ERRO_BASE
            && Math.abs(setpoint.velocity)
                < ConstantesAngulador.VELOCIDADE_MIN;

        if (chegou) {
            anguloHold = goal.position;
            sm.set(StateMachineAngulador.Estado.HOLD);
        }
    }

    private void executarHold() {

        double ffVolts = ff.calculate(
            Math.toRadians(getAngulo()),
            0.0
        );

        io.pid.setSetpoint(
            KInematicsAngulador.grausParaRotacoes(anguloHold),
            SparkBase.ControlType.kPosition,
            ClosedLoopSlot.kSlot0,
            ffVolts
        );
    }


    private void detectarTravamento(double dt) {

        double velocidade =
            Math.abs(
                KInematicsAngulador.rotacoesParaGraus(
                    io.encoder.getVelocity()
                )
            );

        if (sm.is(StateMachineAngulador.Estado.PERFIL)
            && velocidade < ConstantesAngulador.VELOCIDADE_MIN) {

            tempoTravado += dt;
        } else {
            tempoTravado = 0.0;
        }

        if (tempoTravado
            > ConstantesAngulador.TEMPO_MAX_TRAVADO) {

            sm.set(StateMachineAngulador.Estado.FALHA);
            io.motor.stopMotor();
        }
    }
}