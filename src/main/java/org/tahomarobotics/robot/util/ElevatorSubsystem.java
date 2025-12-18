


package org.tahomarobotics.robot.util;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;



import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.littletonrobotics.junction.Logger;
 import org.tahomarobotics.robot.util.AbstractSubsystem;

 import static edu.wpi.first.units.Units.Degrees;
import static org.tahomarobotics.robot.util.ElevatorConstants.*;


    public class ElevatorSubsystem extends AbstractSubsystem {
    private static final ElevatorSubsystem INSTANCE = new ElevatorSubsystem();
    private static final double GEAR_REDUCTION = (12.0 / 72.0) * (30.0 / 60.0);
    public TalonFX right_elevator_motor = new TalonFX(RIGHT_ELEVATOR_MOTOR);
    public TalonFX left_elevator_motor = new TalonFX(LEFT_ELEVATOR_MOTOR);
    boolean isContinuous = true;
    boolean isDiscrete=true;

    public ElevatorSubsystem() {}

    ElevatorState state=ElevatorState.STOWED;
    PositionVoltage posControl=new PositionVoltage(0);
    VelocityDutyCycle velControl = new VelocityDutyCycle(1);

    public Trigger continuousTrigger = new Trigger(()-> isContinuous);

        public Trigger isContinuous() {
            return continuousTrigger;
        }

    @Override
    public void subsystemPeriodic() {
        Logger.recordOutput("ClimberState", state);
        Logger.recordOutput("LeftPosition",left_elevator_motor.getPosition().getValue());
        Logger.recordOutput("RightPosition",right_elevator_motor.getPosition().getValue());
    }


    public void elevatorUp() {
        if (getHeightFt()>9) {
            left_elevator_motor.setControl(velControl.withVelocity(0.5));
            right_elevator_motor.setControl(velControl.withVelocity(0.5));
        }
    }

    public void ElevatorDown() {
        if (getHeightFt() < 1) {
            left_elevator_motor.setControl(velControl.withVelocity(-0.5));
            right_elevator_motor.setControl(velControl.withVelocity(-0.5));
        }
    }


    public static ElevatorSubsystem getInstance() {
    return INSTANCE;
    }


        private final Encoder encoder = new Encoder(0, 1);
        // an encoder is a sensor that will allow us to see its height

    public void transitionToStowed() {
    left_elevator_motor.setControl(posControl.withPosition(Degrees.of(0)));
    right_elevator_motor.setControl(posControl.withPosition(Degrees.of(0)));
    state = ElevatorState.STOWED;
    }

    public double getHeightFt() {
        double rightPos = right_elevator_motor.getPosition().getValueAsDouble();
        double leftPos = left_elevator_motor.getPosition().getValueAsDouble();
        return (rightPos + leftPos) / 2.0;
        // you can't return 2 things from one method so I am getting the average of the height of both the motors

    }


    public void moveDownwardContinuously() {
        if (isContinuous) {
            left_elevator_motor.setControl(velControl.withVelocity(-0.5));
            right_elevator_motor.setControl(velControl.withVelocity(-0.5));
        }
    }

    public void moveUpwardContinuously() {
        if (isContinuous) {
            left_elevator_motor.setControl(velControl.withVelocity(0.5));
            right_elevator_motor.setControl(velControl.withVelocity(0.5));
        }
    }

    public void toggleMode() {
        isContinuous = !isContinuous;
        isDiscrete=!isDiscrete;

    } // the boolean will get reversed

    public void setZeroingVoltage() {
        left_elevator_motor.setVoltage(ZEROING_VOLTAGE);
        right_elevator_motor.setVoltage(ZEROING_VOLTAGE);
    }
    public void zeroPosition() {
        left_elevator_motor.setPosition(ZERO_POSITION);
        right_elevator_motor.setPosition(ZERO_POSITION);
    }

    public void stop() {
        left_elevator_motor.stopMotor();
        right_elevator_motor.stopMotor();
    }

    public void UpPos() {
        left_elevator_motor.setPosition(UP_POSITION);
        right_elevator_motor.setPosition(UP_POSITION);
        state=ElevatorState.UP;
    }

    public void DownPos() {
        left_elevator_motor.setPosition(DOWN_POSITION);
        right_elevator_motor.setPosition(DOWN_POSITION);
        state = ElevatorState.DOWN;
    }



    enum ElevatorState {
        STOWED,
        UP,
        DOWN
    }
}

