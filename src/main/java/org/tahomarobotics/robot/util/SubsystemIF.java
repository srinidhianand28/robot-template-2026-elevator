package org.tahomarobotics.robot.util;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public abstract class SubsystemIF extends SubsystemBase {
    protected final Logger logger;

    protected SubsystemIF() {
        this.logger = LoggerFactory.getLogger(this.getClass());
    }

    // Initialization

    public SubsystemIF initialize() {
        return this;
    }

    public void onDisabledInit() {}

    public void onAutonomousInit() {}

    public void onTeleopInit() {}

    public void onSimulationInit() {}

    // Energy

    public double getEnergyUsed() {
        return 0;
    }

    public double getTotalCurrent() {
        return 0;
    }
}