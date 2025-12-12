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

package org.tahomarobotics.robot.util;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import org.tahomarobotics.robot.ElevatorSubsystem;

import static org.tahomarobotics.robot.util.ElevatorConstants.ZERO_TIME;

public class ZeroElevatorCommand extends Command {
    private final ElevatorSubsystem elevator;
    private final Timer timer=new Timer();

    public ZeroElevatorCommand(ElevatorSubsystem elevator) {
        this.elevator=elevator;
        addRequirements(this.elevator);
    }
    @Override
    public void initialize(){
        timer.restart();
        elevator.setZeroingVoltage();
    }
    @Override
    public boolean isFinished() {
    return timer.hasElapsed(ZERO_TIME);
    }
    @Override
    public void end(boolean interrupted) {
        elevator.left_elevator_motor.setVoltage(0);
        elevator.right_elevator_motor.setVoltage(0);
        elevator.zeroPosition();
    }
   }