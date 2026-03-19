package frc.robot.commands.Autonomo.LimelightAuto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Traction;

public class AlinhadorVerticalAuto extends Command {

    private final Limelight limelight;
    private final Traction traction;
    private final int targetId;

    private static final double KP_DIST = 0.50;
    private static final double DIST_DESEJADA = 2.40;
    private static final double DIST_OK = 0.02;
    private static final double VEL_MAX = 0.6;

    public AlinhadorVerticalAuto(
        Limelight limelight,
        Traction traction,
        int targetId
    ) {
        this.limelight = limelight;
        this.traction = traction;
        this.targetId = targetId;
        addRequirements(traction);
    }

    @Override
    public void initialize() {
        limelight.setPipeline(0);
        limelight.ligarLED();
    }

    @Override
    public void execute() {

        if (!limelight.temAlvo()) {
            traction.stop();
            return;
        }

        int idAtual = limelight.getAprilTagID();
        if (idAtual == -1 || idAtual != targetId) {
            traction.stop();
            return;
        }

        double distancia = limelight.getDistanciaFiltrada();
        if (distancia < 0) {
            traction.stop();
            return;
        }

        double erro = distancia - DIST_DESEJADA;
        double avanco = MathUtil.clamp(
            erro * KP_DIST,
            -VEL_MAX,
            VEL_MAX
        );

        traction.arcadeMode(avanco, 0.0);
    }

    @Override
    public void end(boolean interrupted) {
        traction.stop();
        limelight.desligarLED();
    }

    @Override
    public boolean isFinished() {
        return limelight.temAlvo()
            && limelight.getAprilTagID() == targetId
            && Math.abs(
                limelight.getDistanciaAprilTag() - DIST_DESEJADA
            ) < DIST_OK;
    }
}