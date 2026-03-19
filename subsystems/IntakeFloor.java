package frc.robot.subsystems;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;

import frc.robot.Hardwares.HardwaresIntake;
import frc.robot.Constantes.ConstantesAngulador;
import frc.robot.Constantes.ConstantesIntakeFloor;
import frc.robot.Extras.AngulosPresetPivot;
import frc.robot.StatesMachines.StateMachineIntakeFloor;
import frc.robot.Kinematics.KInematicsAngulador;
import frc.robot.Kinematics.KinematicsIntakeFloor;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

@SuppressWarnings ("unused")
public class IntakeFloor extends SubsystemBase {

    private final HardwaresIntake io = new HardwaresIntake();
    private final StateMachineIntakeFloor sm = new StateMachineIntakeFloor();

    private final ArmFeedforward ff =
        new ArmFeedforward(
            ConstantesIntakeFloor.FFPivot.kS,
            ConstantesIntakeFloor.FFPivot.kG,
            ConstantesIntakeFloor.FFPivot.kV
        );

    private final TrapezoidProfile.Constraints constraints =
        new TrapezoidProfile.Constraints(
            ConstantesIntakeFloor.MAX_VEL_PIVOT,
            ConstantesIntakeFloor.MAX_ACC_PIVOT
        );

    private TrapezoidProfile.State goal = new TrapezoidProfile.State();
    private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();

    private static final double VELOCIDADE_MAX_INTAKE = 0.70;
    private boolean intakeLigado = false;

    private double anguloHoldRad = 0.0;

    public double getAnguloPivotRad() {
        return KinematicsIntakeFloor.rotacoesParaRadPivot(
            io.encoder_pivot.getPosition()
        ) - ConstantesIntakeFloor.OFFSET_PIVOT_RAD;
    }

    public StateMachineIntakeFloor.EstadoPivot getEstadoPivot() {
        return sm.get();
    }

   
    public void moverParaAnguloRad(double anguloRad) {

        double anguloLimitado = MathUtil.clamp(
            anguloRad,
            ConstantesIntakeFloor.LIMITE_INFERIOR_PIVOT,
            ConstantesIntakeFloor.LIMITE_SUPERIOR_PIVOT
        );

        goal = new TrapezoidProfile.State(anguloLimitado, 0.0);
        setpoint = new TrapezoidProfile.State(getAnguloPivotRad(), 0.0);

        sm.set(StateMachineIntakeFloor.EstadoPivot.PERFIL);
    }

    public void moverParaPreset(AngulosPresetPivot preset) {
        moverParaAnguloRad(preset.rad);
    }

    public void IntakeOn() {
        io.IntakeMotor.set(0.6);
        intakeLigado = true;
    }

    public void IntakeReverse() {
        io.IntakeMotor.set(-0.8);
        intakeLigado = true;
    }

    public void PararIntake() {
        io.IntakeMotor.set(0.0);
        intakeLigado = false;
    }

    public boolean isIntakeLigado() {
        return intakeLigado;
    }

    public void forcarHold() {
    anguloHoldRad = getAnguloPivotRad();
    sm.set(StateMachineIntakeFloor.EstadoPivot.HOLD);
}

    @Override
    public void periodic() {

        switch (sm.get()) {
            case PERFIL -> executarPerfil();
            case HOLD   -> executarHold();
            default    -> io.PivotMotor.stopMotor();
        }

        SmartDashboard.putNumber(
            "Pivot/Angulo (graus)",
            Math.toDegrees(getAnguloPivotRad())
        );
        SmartDashboard.putString("Pivot/Estado", sm.get().name());
    }

    private void executarPerfil() { 
        TrapezoidProfile profile = 
            new TrapezoidProfile(constraints); 
                setpoint = profile.calculate(
                     ConstantesIntakeFloor.DT_PIVOT, setpoint, goal 
                     ); 
        double ffVolts =
             ff.calculate( 
                setpoint.position, setpoint.velocity 
            ); 
        io.pid.setSetpoint( 
            KinematicsIntakeFloor.radParaRotacoesPivot(
                setpoint.position
            ),
        SparkBase.ControlType.kPosition, 
        ClosedLoopSlot.kSlot0, ffVolts 
        ); 
    if ((Math.abs(goal.position - getAnguloPivotRad())
        < ConstantesIntakeFloor.MARGEM_ERRO_BASE_PIVOT)) {
         anguloHoldRad = goal.position; 
         sm.set(StateMachineIntakeFloor.EstadoPivot.HOLD); 
        } 
    }

    private void executarHold() {

    double ffVolts = ff.calculate(
        anguloHoldRad,
        0.0
    );

    io.pid.setSetpoint(
        KinematicsIntakeFloor.radParaRotacoesPivot(anguloHoldRad),
        SparkBase.ControlType.kPosition,
        ClosedLoopSlot.kSlot0,
        ffVolts
    );
}
}