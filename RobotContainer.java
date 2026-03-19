package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Traction;
import frc.robot.subsystems.Angulador;
import frc.robot.subsystems.IntakeFloor;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Index;
import frc.robot.commands.Limelight.AlinhadorHorizontalAprilTag;
import frc.robot.commands.Pivot.MoverPivotPreset;
import frc.robot.commands.IntakeFloor.PararIntake;
import frc.robot.commands.IntakeFloor.ToggleIntake;
import frc.robot.commands.IntakeFloor.ToggleIntakeReverse;
import frc.robot.commands.Autonomo.Shooter.BoquinhaAntesShooterAuto;
import frc.robot.commands.Autonomo.Shooter.ShooterLeft;
import frc.robot.commands.Autonomo.Shooter.ShooterRight;
import frc.robot.commands.Autonomo.Tracao.AndarEncoder;
import frc.robot.commands.Autonomo.Tracao.AutoAndarEColetar;
import frc.robot.commands.Autonomo.Tracao.GiroPorAngulo;
import frc.robot.commands.Autonomo.intake.Autobaixointake;
import frc.robot.commands.Index.PararSequencialIndexShooter;
import frc.robot.commands.Index.ToggleSequencialShooterIndex;
import frc.robot.commands.Limelight.AjusteAnguloShooterTraction;
import frc.robot.Constantes.ConstantesShooter;
import frc.robot.Extras.AnguloPreset;
import frc.robot.Extras.AngulosPresetPivot;
import frc.robot.commands.Angulador.AnguladorAutoPorLimelight;
import frc.robot.commands.Angulador.MoverAnguladoPreset;
import frc.robot.commands.Shooter.*;
import frc.robot.commands.Traction.AtivarTurbo;
import frc.robot.commands.Traction.Controller;
import frc.robot.commands.Autonomo.intake.AutoIntakeFloor;
import frc.robot.commands.Autonomo.Angulador.AnguladorAuto;
import frc.robot.commands.Autonomo.Auto.AutoShoot;
import frc.robot.commands.Autonomo.LimelightAuto.AlinhadorHorizontalAuto;
import frc.robot.commands.Autonomo.Shooter.AutoAtirar;
import frc.robot.commands.Autonomo.Shooter.AutoShootCompleto;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;



@SuppressWarnings("unused")
public class RobotContainer {

    /* ===== SUBSYSTEMS ===== */
    private final Shooter shooter = new Shooter();
    private final Traction traction = new Traction();
    private final Angulador angulador = new Angulador();
    private final Limelight limelight = new Limelight();
    private final IntakeFloor intakeFloor = new IntakeFloor();
    private final Index index = new Index();
    //private final Climber climber = new Climber(); 


    private final SendableChooser<Command> autonomousChooser = new SendableChooser<>();


    /* ===== CONTROLES ===== */
    private final XboxController xbox1 = new XboxController(0);
    private final XboxController xbox2 = new XboxController(1);

    /* ===== BOTOES ===== */
    private final JoystickButton btnTurbo =
        new JoystickButton(xbox1, XboxController.Button.kA.value);

    // LIMELIGHT
    private final JoystickButton rb =
        new JoystickButton(xbox2, XboxController.Button.kRightBumper.value);

    private final JoystickButton lb =
        new JoystickButton(xbox2, XboxController.Button.kLeftBumper.value);

    private final Trigger rt =
    new Trigger(() -> xbox2.getRightTriggerAxis() > 0.2);

    private final Trigger lt =
        new Trigger(() -> xbox2.getLeftTriggerAxis() > 0.2);

    // SHOOTER
    private final JoystickButton btnA =
        new JoystickButton(xbox2, XboxController.Button.kA.value);


    private final JoystickButton btnX =
        new JoystickButton(xbox2, XboxController.Button.kX.value);

    private final JoystickButton btnB =
        new JoystickButton(xbox2, XboxController.Button.kB.value);

    private final JoystickButton btnY =
        new JoystickButton(xbox2, XboxController.Button.kY.value);
    private final JoystickButton btnRb = new JoystickButton(xbox1, 6);
    private final JoystickButton btnLb = new JoystickButton(xbox1, 5);
    

    private final JoystickButton btnJ = new JoystickButton(xbox2, 9);

    public RobotContainer() {
        configureBindings();
        configureAutonomous();

        traction.setDefaultCommand(
            new Controller(traction, xbox1)
        );
    }

    private void configureBindings() {

  
        btnTurbo.onTrue(new AtivarTurbo(traction));

        /*btnRb.whileTrue(
            new AlinhadorHorizontalAprilTag(limelight, traction )
        );*/
        btnRb.whileTrue(new AjusteAnguloShooterTraction(limelight, traction, shooter, angulador));

        btnLb.whileTrue(
            new AutoAimShooter( angulador, shooter, limelight)
        );

        btnA.onTrue(new ShooterAutoPorDistancia(shooter, limelight));
 
         btnX.onTrue(
    new ShooterVelocidade(
        shooter,
        ConstantesShooter.Velocidade.MEDIA
    )
);

btnB.whileTrue(
    new ShooterVelocidade(
        shooter,
        ConstantesShooter.Velocidade.ALTA
    )
);

btnJ.whileTrue(
    new AutoAimShooter(angulador, shooter, limelight)
);

        rt.onTrue(new ToggleSequencialShooterIndex(shooter, index));

        lt.onTrue(
            new PararSequencialIndexShooter(shooter, index));

        rb.onTrue(new ToggleIntakeReverse(intakeFloor) );
        lb.onTrue(new ToggleIntake(intakeFloor));

    new POVButton(xbox2, 180)
        .whileTrue(new MoverPivotPreset(
            intakeFloor,
            AngulosPresetPivot.ALTO
        ));
    new POVButton(xbox2, 0)
        .whileTrue(new MoverPivotPreset(
            intakeFloor,
            AngulosPresetPivot.BAIXO
        ));
    new POVButton(xbox2, 270)
        .whileTrue(new MoverPivotPreset(
            intakeFloor,
            AngulosPresetPivot.MEDIO
        ));

    new POVButton(xbox1, 0)
        .onTrue(new MoverAnguladoPreset(
            angulador,
            AnguloPreset.ALTO
        ));

    new POVButton(xbox1, 270)
        .onTrue(new MoverAnguladoPreset(
            angulador,
            AnguloPreset.CENTRAL
        ));

    new POVButton(xbox1, 180)
        .onTrue(new MoverAnguladoPreset(
            angulador,
            AnguloPreset.BAIXO
        ));
    }

    private void configureAutonomous() {
        // Adiciona opções ao chooser
        autonomousChooser.setDefaultOption("Autnoomo Meio", getAutonomousCommandMeio());
        autonomousChooser.addOption("Autonomo Esquerda", getAutonomousCommandEsquerda());
        autonomousChooser.addOption("Autonomo Direita", getAutonomousCommandDireita());
        // Você pode adicionar mais autônomos aqui se quiser

        // Envia para o SmartDashboard
        SmartDashboard.putData("Autonomous Mode", autonomousChooser);
    }
    
    public Command getSelectedAutonomous() {
        return autonomousChooser.getSelected();
    }

   private Command getAutonomousCommandMeio() {
        return new SequentialCommandGroup(
            new AndarEncoder(traction, 0.7, 0.05),
            new GiroPorAngulo(traction, -60),
            new AlinhadorHorizontalAprilTag(limelight, traction).withTimeout(3),
            new AutoIntakeFloor(intakeFloor),
            new BoquinhaAntesShooterAuto(shooter, index, index, intakeFloor),
            new AndarEncoder(traction, 0.7, 0.4),
            new GiroPorAngulo(traction, -25)
        );
    }

    private Command getAutonomousCommandEsquerda() {
        return new SequentialCommandGroup(
        new AndarEncoder(traction, -0.7, 0.20),
        new AutoIntakeFloor(intakeFloor),
        new AutoAndarEColetar(traction, intakeFloor).withTimeout(4),
        new AndarEncoder(traction, 0.7, 0.29),
        new GiroPorAngulo(traction, 8),
        new AlinhadorHorizontalAprilTag(limelight, traction).withTimeout(3),
        new ShooterLeft(shooter, index, index, intakeFloor)

        );
    }
    

    private Command getAutonomousCommandDireita() {
        return new SequentialCommandGroup(
        new AndarEncoder(traction,  0.7, 0.10),
        new GiroPorAngulo(traction, -5.5),
        new AutoIntakeFloor(intakeFloor),
        new ShooterRight(shooter, index, index, intakeFloor)
        );
    }

}

   /* Autonomo LEFT
    public Command getAutonomousCommand() {
        return new SequentialCommandGroup(
        new AndarEncoder(traction, -0.7, 0.05),    
        new AlinhadorHorizontalAprilTag(limelight, traction).withTimeout(3),
        new Autobaixointake(intakeFloor),
        new BoquinhaAntesShooterAuto(shooter, index, index, intakeFloor)

        );
    }   */

    
    /*  AUTONOMO  MEIO 
    public Command getAutonomousCommand() {
        return new SequentialCommandGroup(

        new AndarEncoder(traction, 0.7, 0.06),
        new GiroPorAngulo(traction, -60),
        new AlinhadorHorizontalAprilTag(limelight, traction).withTimeout(3),
        new AutoIntakeFloor(intakeFloor),
        new BoquinhaAntesShooterAuto(shooter, index, index, intakeFloor),
        new AndarEncoder(traction, 0.7, 0.4),
        new GiroPorAngulo(traction, -25)
       );
    } */



