package org.tahomarobotics.robot.chassis.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import org.tahomarobotics.robot.chassis.Chassis;
import org.tahomarobotics.robot.chassis.ChassisConstants;

import java.util.function.DoubleSupplier;

public class TeleopDriveCommand extends Command {
    private final Chassis chassis = Chassis.getInstance();

    private final ChassisSpeeds velocityInput = new ChassisSpeeds();
    private final DoubleSupplier x, y, omega;

    public TeleopDriveCommand(DoubleSupplier x, DoubleSupplier y, DoubleSupplier omega) {
        this.x = x;
        this.y = y;
        this.omega = omega;

        addRequirements(chassis);
    }

    @Override
    public void execute() {
        velocityInput.vxMetersPerSecond = x.getAsDouble() * ChassisConstants.MAX_VELOCITY;
        velocityInput.vyMetersPerSecond = y.getAsDouble() * ChassisConstants.MAX_VELOCITY;
        velocityInput.omegaRadiansPerSecond = omega.getAsDouble() * ChassisConstants.MAX_ANGULAR_VELOCITY;

        chassis.drive(velocityInput);
    }
}
