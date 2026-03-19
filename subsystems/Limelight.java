package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DriverStation;

import frc.robot.Constantes.ConstantesLimelight;

@SuppressWarnings("unused")
public class Limelight extends SubsystemBase {
    
    private final NetworkTable table;

    private double txFiltrado = 0.0;
    private double distFiltrada = 0.0;
    private double UltimaDistanciaValida = 0.0;
    private double ultimoDashboard = 0.0;  
    
    private static final double ALPHA = 0.2; 
    private static final double Tan_Angulo_Minimo = 1e-3;
 
    public Limelight() {
        table = NetworkTableInstance.getDefault().getTable("limelight");
    }

    public boolean temAlvo() {
        return table.getEntry("tv").getDouble(0.0) == 1.0;
    }

    public double getTx() {
        return table.getEntry("tx").getDouble(0.0);
    }

    public double getTy() {
        return table.getEntry("ty").getDouble(0.0);
    }

    public double getTxAlvo () {
        return temAlvo() ? getTx() : Double.NaN;
    }

    public double getTyAlvo () {
        return temAlvo() ? getTy() : Double.NaN;
    }


    public double getTxFiltrado() {
    if (!temAlvo()) {
        return txFiltrado;
    }

    double tx = getTx();
    txFiltrado = (1 - ConstantesLimelight.LimelightConstants.ALPHA_TX) * txFiltrado + ConstantesLimelight.LimelightConstants.ALPHA_TX * tx;
    return txFiltrado;
}

    public double getTxComOffset() {

    if (!temAlvo()) {
        return txFiltrado;
    }

    int id = getAprilTagID();
    double tx = getTxFiltrado();
    double distancia = getDistanciaFiltrada();

    if (id < 0 || distancia <= 0.05 || Double.isNaN(distancia)) {
        return tx;
    }

    double offsetMetros = switch (id) {
        case 11 ->  0.10;
        case 2  -> -0.9;
        case 3  -> -0.20;
        case 4  -> -0.40;
        case 8  ->  0.05;
        case 5, 9 -> 0.10;
        default ->  0.0;
    };

    double offsetGraus =
        Math.toDegrees(Math.atan(offsetMetros / distancia));

    return tx + offsetGraus;
}

    public double getTxShooter() {
    if (!temAlvo()) {
        return txFiltrado;
    }

    double distancia = getDistanciaFiltrada();
    if (distancia < 0.1 || Double.isNaN(distancia)) {
        return txFiltrado;
    }

    double offsetGraus = Math.toDegrees(
        Math.atan(ConstantesLimelight.LimelightConstants.OFFSET_SHOOTER_LATERAL_METROS / distancia)
    );

    return getTxFiltrado() + offsetGraus;
    }

    public boolean alinhadoComShooter() {
    return Math.abs(getTxShooter()) < ConstantesLimelight.LimelightConstants.DEADZONE_TX_GRAUS;
}

    public double getDistanciaAprilTag() {
        if (!temAlvo()) {
            return Double.NaN;
        }

    

        double ty = getTy();
        double anguloTotalGraus = ConstantesLimelight.LimelightConstants.ANGULO_CAMERA_EFETIVO_GRAUS + ty;
        double anguloTotalRad = Math.toRadians(anguloTotalGraus);

        double tangente = Math.tan(anguloTotalRad);
        if (Math.abs(tangente) < Tan_Angulo_Minimo) {
            DriverStation.reportWarning(
                "Limelight: tan(angulo) muito pequeno, distancia invalida. Angulo=" + anguloTotalGraus,
                false
            );
            return UltimaDistanciaValida == 0.0 ? Double.NaN : UltimaDistanciaValida;
        }


        double distanciaCamera =
        (ConstantesLimelight.LimelightConstants.ALTURA_TAG_METROS - ConstantesLimelight.LimelightConstants.ALTURA_CAMERA_METROS) / tangente;


        double distanciaBumper = distanciaCamera - ConstantesLimelight.LimelightConstants.OFFSET_CAMERA_BUMPER_METROS;
        return Math.max(distanciaBumper, 0.0);
    }


        public double getDistanciaFiltrada() {
            double d = getDistanciaAprilTag();
            if (Double.isNaN(d)) {
                return UltimaDistanciaValida;
            }
            distFiltrada = (1 - ALPHA) * distFiltrada + ALPHA * d;
            UltimaDistanciaValida = distFiltrada;
            return distFiltrada;
        }

        public void ligarLED() {
            table.getEntry("ledMode").setNumber(3);
        }

        public void desligarLED() {
            table.getEntry("ledMode").setNumber(1);
        }

        public int getAprilTagID() {
            return (int) table.getEntry("tid").getDouble(-1);
        }
        public void setPipeline(int pipeline) {
            table.getEntry("pipeline").setNumber(pipeline);
        }
        @Override
        public void periodic() {
            double agora = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
            if (agora - ultimoDashboard >= 0.2) {
            ultimoDashboard = agora;
            SmartDashboard.putBoolean("Limelight/Tem Alvo", temAlvo());
            SmartDashboard.putNumber("Limelight/tx (graus)", getTx());
            SmartDashboard.putNumber("Limelight/ty (graus)", getTy());
            SmartDashboard.putNumber("Limelight/Distancia (m)", getDistanciaAprilTag());
            SmartDashboard.putNumber("limelight/Distancia Filtrada (m)", getDistanciaFiltrada());
            }
        }
    }
