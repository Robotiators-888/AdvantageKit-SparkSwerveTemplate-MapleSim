// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Utility class for creating triggers based on the robot's mode. */
public class RobotModeTriggers {

    /** @return A trigger that is active when the robot is in teleop mode. */
    public static Trigger teleop() {
        return new Trigger(DriverStation::isTeleopEnabled);
    }

    /** @return A trigger that is active when the robot is disabled. */
    public static Trigger disabled() {
        return new Trigger(DriverStation::isDisabled);
    }
}
