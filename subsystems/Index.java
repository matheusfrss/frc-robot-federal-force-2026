package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Hardwares.HardwaresIndex;

public class Index extends SubsystemBase {

    private final HardwaresIndex io = new HardwaresIndex();

    public void ligar() {
        io.boquinha.set(0.39); 
        io.index.set(0.60);
    }
    public void ligarBoquinha() {
        io.boquinha.set(0.35);
    }

    public void desligar() {
        io.boquinha.stopMotor();
        io.index.stopMotor();
    }
}