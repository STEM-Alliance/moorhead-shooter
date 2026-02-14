// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class ShooterConstants {
    public static final int HOOD_MOTOR_PORT = 1;
    public static final int SHOOTER_LEADER_PORT = 2;
    public static final int SHOOTER_FOLLOWER_PORT = 3;

    public static final boolean SHOOTER_LEADER_INVERTED = false;
    public static final boolean SHOOTER_FOLLOWER_INVERTED = true;
    public static final boolean HOOD_INVERTED = false;
    public static final double SHOOTER_MAX_RPM = 6500.0;

    public static final double HOOD_GEAR_RATIO = (12d / 48d) * (18d / 310d);
    public static final double HOOD_MIN_ANGLE = 0;
    public static final double HOOD_MAX_ANGLE = 32.982;

    public static final double SHOOTER_P = 0.0012;
    public static final double SHOOTER_I = 0.000;
    public static final double SHOOTER_D = 0.0000;

    public static final PIDController HOOD_PID = new PIDController(0.0165, 0.0, 0.0001);
    

  }
}
