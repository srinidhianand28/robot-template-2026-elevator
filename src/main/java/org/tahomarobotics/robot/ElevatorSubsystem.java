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

package org.tahomarobotics.robot;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;



import edu.wpi.first.wpilibj.Encoder;
import org.tahomarobotics.robot.util.AbstractSubsystem;

import static edu.wpi.first.units.Units.Feet;

import static org.tahomarobotics.robot.util.ElevatorConstants.LEFT_ELEVATOR_MOTOR;
import static org.tahomarobotics.robot.util.ElevatorConstants.RIGHT_ELEVATOR_MOTOR;

public class ElevatorSubsystem extends AbstractSubsystem {
    private static final ElevatorSubsystem INSTANCE = new ElevatorSubsystem();
    private static final double GEAR_REDUCTION = (12.0 / 72.0) * (30.0 / 60.0);
    TalonFX right_elevator_motor = new TalonFX(RIGHT_ELEVATOR_MOTOR);
    TalonFX left_elevator_motor = new TalonFX(LEFT_ELEVATOR_MOTOR);
    boolean isContinous = true;
    public ElevatorSubsystem() {}

    @Override
    public void subsystemPeriodic() {

    }


    public void elevatorUp() {
        if (getHeightFt()>9) {
            left_elevator_motor.setControl(new DutyCycleOut(0.5));
            right_elevator_motor.setControl(new DutyCycleOut(0.5));
        }
    }

    public void ElevatorDown() {
        if (getHeightFt() < 1) {
            left_elevator_motor.setControl(new DutyCycleOut(-0.5));
            right_elevator_motor.setControl(new DutyCycleOut(-0.5));
        }
    }


    public static ElevatorSubsystem getInstance() {
    return INSTANCE;
    }


        private final Encoder encoder = new Encoder(0, 1);
        // an encoder is a sensor that will allow us to see its height



    public double getHeightFt() {
        double rightPos = right_elevator_motor.getPosition().getValueAsDouble();
        double leftPos = left_elevator_motor.getPosition().getValueAsDouble();
        return (rightPos + leftPos) / 2.0;
        // you can't return 2 things from one method so I am getting the average of the height of both the motors

    }


    public void moveDownwardContinuously() {
        if (isContinous) {
            left_elevator_motor.setControl(new DutyCycleOut(-0.5));
            right_elevator_motor.setControl(new DutyCycleOut(-0.5));
        }
    }

    public void moveUpwardContinuously() {
        if (isContinous) {
            left_elevator_motor.setControl(new DutyCycleOut(0.5));
            right_elevator_motor.setControl(new DutyCycleOut(0.5));
        }
    }

    public void toggleMode() {
        isContinous = !isContinous;
    } // the boolean will get reversed

}
