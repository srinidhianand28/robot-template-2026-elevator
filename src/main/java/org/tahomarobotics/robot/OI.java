package org.tahomarobotics.robot;


import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.tinylog.Logger;


import java.util.List;
import java.util.Set;
import java.util.function.Function;


public class OI {


    // -- Constants --


    private static final double ROTATIONAL_SENSITIVITY = 2.0;
    private static final double TRANSLATIONAL_SENSITIVITY = 1.3;


    private static final double DEADBAND = 0.09;
    private static final double TRIGGER_DEADBAND = 0.05;


    private final CommandXboxController controller = new CommandXboxController(0);
    private final CommandXboxController lessImportantController = new CommandXboxController(1);


    public OI(RobotContainer robotContainer) {
        DriverStation.silenceJoystickConnectionWarning(true);


        configureControllerBindings();
        configureLessImportantControllerBindings();


        setDefaultCommands();
    }


    // -- Bindings --


    public void configureControllerBindings() {
    controller.y().onTrue(elevator.goUp());
    controller.a().onTrue(elevator.goDown());
    controller.b().onTrue(elevator.toggle());
    controller.b().whileTrue(elevator.toggle());
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




