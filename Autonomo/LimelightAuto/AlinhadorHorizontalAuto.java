package frc.robot.commands.Autonomo.LimelightAuto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Traction;

public class AlinhadorHorizontalAuto extends Command {

    private final Limelight limelight;
    private final Traction traction;
    private final int targetId;

    private static final double KP_ROT = 0.035;
    private static final double ANGULO_OK = 0.5; // graus

    public AlinhadorHorizontalAuto(
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

        double erroX = limelight.getTxFiltrado();
        double rot = Math.abs(erroX) > ANGULO_OK
            ? erroX * KP_ROT
            : 0.0;

        rot = MathUtil.clamp(rot, -0.8, 0.8);
        traction.arcadeMode(0.0, rot);
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
            && Math.abs(limelight.getTx()) < ANGULO_OK;
    }
}