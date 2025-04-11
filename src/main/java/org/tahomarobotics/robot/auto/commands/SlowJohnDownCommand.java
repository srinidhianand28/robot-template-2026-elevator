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

package org.tahomarobotics.robot.auto.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import org.tahomarobotics.robot.Robot;
import org.tahomarobotics.robot.chassis.Chassis;
import org.tinylog.Logger;

public class SlowJohnDownCommand extends Command {
    private final Chassis chassis = Chassis.getInstance();

    private double velocity;
    private final Rotation2d direction;
    private boolean wasNegative = false;
    private final static double DECEL = 2;
    private double timeout;
    private Timer timer = new Timer();

    public SlowJohnDownCommand(Rotation2d direction) {
        this.direction = direction;

        addRequirements(chassis);
    }

    @Override
    public void initialize() {
        timer.restart();
        velocity = getVelocity();
        wasNegative = velocity < 0;
        timeout = velocity / DECEL;

        if (wasNegative) {
            Logger.info("john movin' negative; slowing down");
        }
    }

    @Override
    public void execute() {
        if (!wasNegative) { return; }
        velocity = velocity + (DECEL * Robot.defaultPeriodSecs);
        chassis.drive(new ChassisSpeeds(velocity * direction.getCos(), velocity * direction.getSin(), 0), true);
    }

    public double getVelocity() {
        var v = chassis.getFieldChassisSpeeds();
        return v.vxMetersPerSecond * direction.getCos() + v.vyMetersPerSecond * direction.getSin();
    }

    @Override
    public boolean isFinished() {
        return !wasNegative || timer.hasElapsed(timeout + 0.05) || velocity > -1e-2;
    }

    @Override
    public void end(boolean interrupted) {
        if (wasNegative) {
            chassis.drive(new ChassisSpeeds(0, 0, 0), true);
            Logger.info("john slowed down");
        }
    }
}
