package frc.robot.commands.autocommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.FieldLocation;
import frc.robot.commands.carouselcommands.AutoSpeedCarouselCommand;
import frc.robot.commands.feedercommands.AutoFeederCommand;
import frc.robot.commands.shootcommands.ShootSetupCommand;
import frc.robot.subsystems.CarouselSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.HoodSubsystem;

public class ShootCG extends ParallelCommandGroup {

        /**
         * Shoot balls
         */
        public ShootCG(FlywheelSubsystem flywheelSubsystem, HoodSubsystem hoodSubsystem,
                        FeederSubsystem feederSubsystem, CarouselSubsystem carouselSubsystem) {
                // Spin up the flywheel and set the hood, spin the feeder and run the carousel
                // when the feeder starts
                // Spin flywheel and set hood
                Command shootSetup = new ShootSetupCommand(flywheelSubsystem, hoodSubsystem,
                                () -> FieldLocation.INITLINE).withTimeout(10);
                // Spin feeder and end when ready to shoot
                Command feeder = parallel(
                                new AutoFeederCommand(feederSubsystem, () -> carouselSubsystem.atOpenSpace(),
                                                () -> flywheelSubsystem.atSetpoint()),
                                sequence(new WaitCommand(4),
                                                new AutoSpeedCarouselCommand(carouselSubsystem,
                                                                () -> flywheelSubsystem.getSetpoint())))
                                                                                .withTimeout(10);
                addCommands(shootSetup, feeder);

        }
}