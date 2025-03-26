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

package org.tahomarobotics.robot.windmill;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public class Dynamics {

    public record Voltages(double elevatorVoltage, double armVoltage) {}

    private static double lbsInchSqToKilogramMeterSq(double value) {
        return Units.lbsToKilograms(Units.inchesToMeters(Units.inchesToMeters(value)));
    }
    private static final double g = 9.80665;
    private static final double me = Units.lbsToKilograms(17.603);
    private static final double ma = Units.lbsToKilograms(5.947);
    private static final double lc_empty = Units.inchesToMeters(12.241);
    private static final double lc_coral = Units.inchesToMeters(15.655);
    private static final double I_empty = lbsInchSqToKilogramMeterSq(606.25);
    private static final double I_coral = lbsInchSqToKilogramMeterSq(844.14);
    private static final double coral = Units.lbsToKilograms(0.913);

    private DCMotor elevatorMotors = DCMotor.getKrakenX60Foc(2)
                                            .withReduction(1 / WindmillConstants.ELEVATOR_GEAR_REDUCTION / WindmillConstants.ELEVATOR_MAIN_PULLEY_RADIUS);

    private DCMotor armMotor = DCMotor.getKrakenX60Foc(1)
                                      .withReduction(1 / WindmillConstants.ARM_ROTOR_TO_ENCODER /
                                          WindmillConstants.ARM_BELT_REDUCTION);

    // ma, lc and I will change with coral
    private enum K {

        EMPTY(me, ma, lc_empty, I_empty),
        CORAL(me, ma + coral, lc_coral, I_coral);

        final double me_ma;
        final double ma_lc;
        final double ma_lc2_I;

        K(double me, double ma, double lc, double I) {
            this.me_ma = me + ma;
            this.ma_lc = ma * lc;
            this.ma_lc2_I = ma * lc * lc * I;
        }
    }


    public Voltages inverseDynamics(WindmillState state, double robotAcceleration, boolean hasCoral) {

        WindmillState.ElevatorState elevatorState = state.elevatorState();
        WindmillState.ArmState armState = state.armState();

        double sinTheta = Math.sin(armState.angleRadians());
        double cosTheta = Math.cos(armState.angleRadians());

        // select constants base on elevatorState.position and if coral is attached
        K k = hasCoral ? K.CORAL : K.EMPTY;

        // forces due to acceleration
        double elevatorForce =
            k.me_ma * elevatorState.accelerationMetersPerSecondSquared() +
            k.ma_lc * cosTheta * armState.accelerationRadiansPerSecondSquared();
        double armTorque =
            -k.ma_lc * sinTheta * robotAcceleration +
            k.ma_lc * cosTheta * elevatorState.accelerationMetersPerSecondSquared() +
            k.ma_lc2_I * armState.accelerationRadiansPerSecondSquared();

        // forces due to velocity
        elevatorForce += -k.ma_lc * armState.velocityRadiansPerSecond() * armState.velocityRadiansPerSecond() * sinTheta;

        // forces due to gravity
        elevatorForce += k.me_ma * g;
        armTorque += k.ma_lc * g * cosTheta;

        // elevatorMotors has the pulley radius (m) integration so torque (Nm) and rotational velocity (rad/s)
        // can be replaced with force (N) and linear velocity (m/s)
        Voltages voltages = new Voltages(
            elevatorMotors.getVoltage(elevatorForce, elevatorState.velocityMetersPerSecond()),
            armMotor.getVoltage(armTorque, armState.velocityRadiansPerSecond())
        );

        return voltages;
    }
}
