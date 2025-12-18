package org.tahomarobotics.robot;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.tahomarobotics.robot.util.Elevator;
import org.tahomarobotics.robot.util.ElevatorSubsystem;

import java.util.function.BooleanSupplier;


public class OI extends ElevatorSubsystem {


    // -- Constants --


    private static final double ROTATIONAL_SENSITIVITY = 2.0;
    private static final double TRANSLATIONAL_SENSITIVITY = 1.3;


    private static final double DEADBAND = 0.09;
    private static final double TRIGGER_DEADBAND = 0.05;


    private final CommandXboxController controller = new CommandXboxController(0);

    private final CommandXboxController lessImportantController = new CommandXboxController(1);
    private final Elevator elevator;





    public OI(RobotContainer elevator) {
        this.elevator = new Elevator();
        configureControllerBindings();
    }



    public void configureControllerBindings() {

        controller.y().and(continuousTrigger).onTrue(elevator.goUp());
        controller.y().and(continuousTrigger.negate()).onTrue(elevator.upPosition());
    controller.a().and(continuousTrigger).onTrue(elevator.goDown());
    controller.a().and(continuousTrigger.negate()).onTrue(elevator.downPosition());
    controller.b().onTrue(elevator.toggle());
    // make trigger for continuous and discrete
    }




    public void configureLessImportantControllerBindings() {
    }


    @SuppressWarnings("SuspiciousNameCombination")
    public void setDefaultCommands() {
    }


    // -- Inputs --


    public double getLeftX() {
        return -desensitizePowerBased(controller.getLeftX(), TRANSLATIONAL_SENSITIVITY);
    }


    public double getLeftY() {
        return -desensitizePowerBased(controller.getLeftY(), TRANSLATIONAL_SENSITIVITY);
    }


    public double getRightX() {
        return -desensitizePowerBased(controller.getRightX(), ROTATIONAL_SENSITIVITY);
    }


    // -- Helper Methods --


    public double desensitizePowerBased(double value, double power) {
        value = MathUtil.applyDeadband(value, DEADBAND);
        value *= Math.pow(Math.abs(value), power - 1);
        return value;
    }
}




