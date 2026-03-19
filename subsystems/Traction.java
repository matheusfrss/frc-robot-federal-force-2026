package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
//import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Hardwares.HardwaresTraction;

public class Traction extends SubsystemBase {

    // VARIAVEIS
    public boolean turbo;
    private double ultimoPrint = 0;
        // Constantes físicas
    private static final double WHEEL_DIAMETER_METERS = 0.1524; // 6 polegadas
    private static final double METERS_PER_ROTATION =
            Math.PI * WHEEL_DIAMETER_METERS;

    // Ajuste do robô
    private static final double GEAR_RATIO = 10.71;
    private final Pigeon2 pigeon = new Pigeon2(22);

    private final HardwaresTraction io = new HardwaresTraction();

    // Drive
    private DifferentialDrive differentialDrive =
            new DifferentialDrive(io.leftMotorControllerGroup, io.rightMotorControllerGroup);

    // CONTROLE DE MOVIMENTO

    public void arcadeMode(double drive, double turn) {
        differentialDrive.arcadeDrive(drive, +turn);
    }

    public void stop() {
        differentialDrive.stopMotor();
    }

    public void ativarTurbo(boolean turbo) {
        this.turbo = turbo;
    }

    // GYRO


    public void resetYaw() {
        pigeon.setYaw(0);
    }

    public double getYaw() {
        return pigeon.getRotation2d().getDegrees();
    }


    @Override
public void periodic() {
    double agora = Timer.getFPGATimestamp();

    if (agora - ultimoPrint>= 0.2) {
        ultimoPrint = agora;

        double yaw = pigeon.getYaw().getValueAsDouble();
        double rateZ = pigeon.getAngularVelocityZWorld().getValueAsDouble();

        /*DriverStation.reportWarning(
            "PIGEON | Yaw=" + yaw + " | RateZ=" + rateZ,
            false
        );*/
        SmartDashboard.putNumber("Pigeon/Yaw", yaw);
        SmartDashboard.putNumber("Pigeon/RateZ", rateZ);

      double leftDistance =
    (io.leftEncoder.getPosition() / GEAR_RATIO) * METERS_PER_ROTATION;

double rightDistance =
    (-io.rightEncoder.getPosition() / GEAR_RATIO) * METERS_PER_ROTATION;

SmartDashboard.putNumber("Encoder/LeftDistance", leftDistance);
SmartDashboard.putNumber("Encoder/RightDistance", rightDistance);
SmartDashboard.putNumber("Encoder/AverageDistance", getAverageDistance());

    }
}


    // ENCODERS

    public void resetEncoders() {
        io.leftEncoder.setPosition(0);
        io.rightEncoder.setPosition(0);
    }

    public double getAverageDistance() {
        double leftDistance =
            (io.leftEncoder.getPosition() / GEAR_RATIO) * METERS_PER_ROTATION;

        double rightDistance =
            (-io.rightEncoder.getPosition() / GEAR_RATIO) * METERS_PER_ROTATION;


        return (Math.abs(leftDistance) + Math.abs(rightDistance)) / 2.0;
        //(leftDistance + rightDistance) / 2.0;
        }

}