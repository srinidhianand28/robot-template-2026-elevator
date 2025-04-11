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

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import org.tahomarobotics.robot.chassis.Chassis;
import org.tahomarobotics.robot.util.BlendedPath;
import org.tahomarobotics.robot.util.motion.MotionProfile;
import org.tahomarobotics.robot.util.motion.MotionState;
import org.tahomarobotics.robot.util.motion.TrapezoidalMotionProfile;
import org.tahomarobotics.robot.vision.Vision;
import org.tinylog.Logger;

import java.util.Arrays;
import java.util.Objects;

import static org.tahomarobotics.robot.auto.AutonomousConstants.*;

public class DriveToPoseV99Command extends Command {

    private final Chassis chassis = Chassis.getInstance();

    private final double blendingDistance;

    private BlendedPath path;

    private final Timer timer;
    private Pose2d initialPose;
    private Pose2d goalPose;
    private Pose2d[] waypoints;

    private final PIDController crossTrackController, alongTrackController;
    private MotionProfile profile;
    private final MotionState setpoint;
    private boolean failed = false;
    private Rotation2d direction;
    private boolean profileCompleted;
    private final ProfiledPIDController r;
    private final int isolationTarget;
    private Translation2d point;

    private boolean maintainVelocity = false;

    private static final double TIMEOUT = 2;

    public DriveToPoseV99Command(int isolationTarget, double blendingDistance, Pose2d... waypoints) {
        this.isolationTarget = isolationTarget;
        this.waypoints = Arrays.stream(waypoints).filter(Objects::nonNull).toArray(Pose2d[]::new);
        this.blendingDistance = blendingDistance;

        timer = new Timer();

        crossTrackController = new PIDController(8, 0, 0.5);
        alongTrackController = new PIDController(4, 0, 0);

        setpoint = new MotionState();
        addRequirements(chassis);

        r = new ProfiledPIDController(
            ROTATION_ALIGNMENT_KP, ROTATION_ALIGNMENT_KI, ROTATION_ALIGNMENT_KD,
            ROTATION_ALIGNMENT_CONSTRAINTS
        );
        r.setTolerance(ROTATION_ALIGNMENT_TOLERANCE);
        r.enableContinuousInput(-Math.PI, Math.PI);

    }


    @Override
    public void initialize() {

        // initialize state
        profileCompleted = false;
        failed = false;
        timer.restart();

        // create path from current pose to score pose and approach
        initialPose = chassis.getPose();
        point = initialPose.getTranslation();
        Translation2d[] wp = new Translation2d[1 + waypoints.length];
        wp[0] = initialPose.getTranslation();
        for (int i = 0; i < waypoints.length; i++) {
            wp[i + 1] = waypoints[i].getTranslation();
        }
        var p = BlendedPath.create(blendingDistance, wp);
        if (p.isEmpty()) {
            // path too small or something
            failed = true;
            return;
        }
        path = p.get();

        // get current velocity in direction of new path
        goalPose = waypoints[waypoints.length - 1];
        direction = goalPose.getTranslation().minus(initialPose.getTranslation()).getAngle();
        double currentVelocity = velocityAlongPath(chassis.getFieldChassisSpeeds(), direction);

        // create a velocity or motion profile for the path length
        try {
            profile = new TrapezoidalMotionProfile(
                0d, 0d, path.getTotalDistance(), Math.max(currentVelocity, 0), 0d,
                4,
                5
            );
        } catch (MotionProfile.MotionProfileException e) {
            failed = true;
        }


        // reset PID controllers and heading profile
        crossTrackController.reset();
        alongTrackController.reset();

        r.setGoal(goalPose.getRotation().getRadians());
        r.reset(initialPose.getRotation().getRadians(), chassis.getFieldChassisSpeeds().omegaRadiansPerSecond);

        // isolate to reef tag
        chassis.setAutoAligning(true);
        Vision.getInstance().isolate(isolationTarget);


        org.littletonrobotics.junction.Logger.recordOutput("Autonomous/Drive To Pose/Waypoints", waypoints);
    }

    @Override
    public void execute() {

        // skip if something is wrong
        if (failed) {
            return;
        }

        // expected position, velocity and acceleration
        double t = timer.get();
        profileCompleted = !profile.getSetpoint(t, setpoint);

        // expected pose point
        var prevPoint = point;
        point = path.sample(setpoint.position);

        // calculate feed-forward velocity components
        var currentPose = chassis.getPose();
        double vr = r.calculate(currentPose.getRotation().getRadians());

        double vx, vy;

        var deltaPosition = point.minus(prevPoint);
        if (deltaPosition.getNorm() > 1e-6) {
            direction = deltaPosition.getAngle();
        }

        var positionError = currentPose.getTranslation().minus(point);
        var pathError = positionError.rotateBy(direction.unaryMinus());

        double vCrossTrack = crossTrackController.calculate(pathError.getY(), 0d);
        double vAlongTrack = MathUtil.clamp((t / profile.getEndTime() - 0.95) / 0.05, 0, 1) * alongTrackController.calculate(pathError.getX(), 0d);

        var vel = new Translation2d(setpoint.velocity + vAlongTrack, vCrossTrack).rotateBy(direction);
        vx = vel.getX();
        vy = vel.getY();

        org.littletonrobotics.junction.Logger.recordOutput("Autonomous/Drive To Pose/v", setpoint.velocity);
        org.littletonrobotics.junction.Logger.recordOutput("Autonomous/Drive To Pose/vx", vx);
        org.littletonrobotics.junction.Logger.recordOutput("Autonomous/Drive To Pose/vy", vy);
        org.littletonrobotics.junction.Logger.recordOutput("Autonomous/Drive To Pose/target", new Pose2d(point, direction));
        org.littletonrobotics.junction.Logger.recordOutput("Autonomous/Drive To Pose/vcross", vCrossTrack);
        org.littletonrobotics.junction.Logger.recordOutput("Autonomous/Drive To Pose/valong", vAlongTrack);
        org.littletonrobotics.junction.Logger.recordOutput("Autonomous/Drive To Pose/vaalongerr", pathError.getX());
        org.littletonrobotics.junction.Logger.recordOutput("Autonomous/Drive To Pose/vcrosserr", pathError.getY());
        org.littletonrobotics.junction.Logger.recordOutput("Autonomous/Drive To Pose/velocityalongpath", getVelocityAlongPath());

        chassis.drive(new ChassisSpeeds(vx, vy, vr), true);
    }

    public static double velocityAlongPath(ChassisSpeeds speed, Rotation2d direction) {
        return speed.vxMetersPerSecond * direction.getCos() + speed.vyMetersPerSecond * direction.getSin();
    }

    public double getVelocityAlongPath() {
        return velocityAlongPath(chassis.getTargetFieldChassisSpeeds(), direction);
    }

    public double duration() {
        return profile.getEndTime();
    }

    public void maintainVelocity(boolean b) {
        maintainVelocity = b;
    }

    @Override
    public void end(boolean interrupted) {
        if (!maintainVelocity) {
            chassis.drive(new ChassisSpeeds(), true);
        }
        chassis.setAutoAligning(false);

        Vision.getInstance().globalize();

    }

    @Override
    public boolean isFinished() {
        if (failed) {
            Logger.warn("failed DriveToPose");
            return true;
        }
        if (profileCompleted) {
            if (atPosition()) {
                Logger.info("Successful DriveToPose");
                return true;
            }
            if (timer.hasElapsed(profile.getEndTime() + TIMEOUT)) {
                Logger.warn("DriveToPose timed out");
                return true;
            }
        }
        return false;
    }

    private boolean atPosition() {
        Translation2d robotToTarget = chassis.getPose().getTranslation().minus(goalPose.getTranslation());
        return Math.abs(robotToTarget.getX()) < X_TOLERANCE && Math.abs(robotToTarget.getY()) < Y_TOLERANCE && r.atGoal();
    }

    public boolean isInitialized() {
        return timer.isRunning();
    }

    public double distanceToEnd() {
        return chassis.getPose().getTranslation().getDistance(goalPose.getTranslation());
    }
}
