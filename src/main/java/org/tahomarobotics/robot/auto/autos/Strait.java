/*
 * Copyright 2025 Tahoma Robotics
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

package org.tahomarobotics.robot.auto.autos;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.tahomarobotics.robot.auto.AutonomousConstants;
import org.tahomarobotics.robot.auto.commands.DriveToPoseV4Command;
import org.tahomarobotics.robot.collector.Collector;
import org.tahomarobotics.robot.collector.CollectorCommands;
import org.tahomarobotics.robot.grabber.Grabber;
import org.tahomarobotics.robot.indexer.Indexer;
import org.tahomarobotics.robot.windmill.Windmill;
import org.tahomarobotics.robot.windmill.commands.WindmillCommands;
import org.tahomarobotics.robot.windmill.commands.WindmillMoveCommand;
import org.tinylog.Logger;

import java.util.function.DoubleSupplier;

import static org.tahomarobotics.robot.windmill.WindmillConstants.TrajectoryState.L4;
import static org.tahomarobotics.robot.windmill.WindmillConstants.TrajectoryState.STOW;

public class Strait extends SequentialCommandGroup {
    // -- Constants --

    private static final double SCORING_DISTANCE = Units.inchesToMeters(3);
    private static final double ARM_UP_DISTANCE = AutonomousConstants.APPROACH_DISTANCE_BLEND_FACTOR + Units.inchesToMeters(6);
    private static final double SCORING_TIME = 0.25;

    // -- Requirements --

    private final Collector collector = Collector.getInstance();
    private final Grabber grabber = Grabber.getInstance();
    private final Indexer indexer = Indexer.getInstance();
    private final Windmill windmill = Windmill.getInstance();

    // -- Determinants --

    private final DriverStation.Alliance alliance;

    // -- Initialization --

    public Strait(DriverStation.Alliance alliance) {
        setName("Strait");

        this.alliance = alliance;

        Timer timer = new Timer();
        addCommands(
            Commands.runOnce(timer::restart),
            Commands.parallel(
                // Assuming proper starting state, this will take several cycles.
                WindmillCommands.createElevatorZeroCommand(Windmill.getInstance()),
                CollectorCommands.createZeroCommand(Collector.getInstance())
            ),
            // Assuming proper starting state, this will take one cycle.
//            Commands.runOnce(() -> {
//                windmill.calibrate();
//                collector.zero();
//            }),
            // Drive to our first scoring position then score
            driveToPoleThenScore('G'),
            // Reset the robot upon finishing
            Commands.parallel(
                collector.runOnce(() -> {
                    collector.collectorTransitionToDisabled();
                    collector.deploymentTransitionToStow();
                }),
                indexer.runOnce(indexer::transitionToDisabled)
            ),
            WindmillMoveCommand.fromTo(L4, STOW).orElseThrow(),
            Commands.runOnce(grabber::transitionToDisabled),
            Commands.runOnce(() -> Logger.info("Strait completed in {} seconds.", timer.get()))
        );
    }

    public Command driveToPoleThenScore(char pole) {
        return driveToPoleThenScore(pole, () -> 0);
    }

    public Command driveToPoleThenScore(char pole, DoubleSupplier fudge) {
        // Drive to the scoring position
        DriveToPoseV4Command dtp = AutonomousConstants.getObjectiveForPole(pole - 'A', alliance).fudgeY(fudge.getAsDouble()).driveToPoseV4Command();

        // Move arm from STOW to L4
        Command stowToL4 = WindmillMoveCommand.fromTo(STOW, L4).orElseThrow();

        // Score grabber
        Command scoreGrabber = grabber.runOnce(grabber::transitionToScoring).andThen(Commands.waitSeconds(SCORING_TIME)).andThen(grabber::transitionToDisabled);

        Timer timer = new Timer();
        return Commands.parallel(
            Commands.runOnce(timer::restart),
            dtp,
            dtp.runWhen(() -> dtp.getTargetWaypoint() == 0 && dtp.getDistanceToWaypoint() <= ARM_UP_DISTANCE, stowToL4),
            dtp.runWhen(
                () -> dtp.getTargetWaypoint() == 1 && dtp.getDistanceToWaypoint() <= SCORING_DISTANCE,
                scoreGrabber.andThen(Commands.runOnce(() -> Logger.info("Scored grabber.")))
            )
        ).andThen(Commands.runOnce(() -> Logger.info("Driving to {} and scoring took {} seconds.", pole, timer.get())));
    }
}
