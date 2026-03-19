package frc.robot.commands.Autonomo.Shooter;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.IntakeFloor;

import frc.robot.Constantes.ConstantesShooter;

public class ShooterLeft extends SequentialCommandGroup {

    private static final double ATRASO_BOQUINHA = 0.9;
    private static final double ATRASO_INDEX = 1.2
    ;
    private static final double TEMPO_TOTAL = 7.0;

    public ShooterLeft(
    Shooter shooter,
    Index boquinha,
    Index index,
    IntakeFloor intake

) {

        addCommands(

            // Liga shooter
            new InstantCommand(() -> {
                shooter.setVelocidade(ConstantesShooter.Velocidade.AUTOLEFT);
                shooter.atirarFrente();
            }, shooter),

            // Espera antes da boquinha
            new WaitCommand(ATRASO_BOQUINHA),

            // Liga boquinha
            new InstantCommand(() -> {
                index.ligar();
            }, boquinha),

            // Espera antes do index
            new WaitCommand(ATRASO_INDEX),

            // Liga index
            new InstantCommand(() -> {
                index.ligar();
            }, index),
   
            new InstantCommand(intake::IntakeOn, intake),

            //  Mantem tudo rodando pelo tempo restante
            new WaitCommand(TEMPO_TOTAL - ATRASO_BOQUINHA - ATRASO_INDEX),

            // Para tudo
            new InstantCommand(() -> {
                shooter.parar();
                index.desligar();
                intake.PararIntake();
            }, shooter, boquinha, index, intake)
        );
    }
}