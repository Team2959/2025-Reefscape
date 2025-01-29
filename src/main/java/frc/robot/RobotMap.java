// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

public final class RobotMap {

    // Swerve Motor Assemblies - SparkMax
    // Drive motor CAN == module #
    // Steer motor CAN == module # + 10
    // CAN coder CAN == module #
    public static final int kFrontLeftModule = 4;
    public static final int kBackLeftModule = 2;
    public static final int kBackRigvhtModule = 3;
    public static final int kFrontRightModule = 1;
    // unused module - 5

    // CAN motor addresses
        // SparkMax
 
        // Victor SPX
 
        // Talon SRX

        // Amp

    // PWM motor addresses
        // Servos

    // Analog Input addresses

    // REV Pneumatic Hub solenoid addresses

    // Digital IO addresses
  

    // Operator input USB ports
    public static final int kLeftJoystick = 0;
    public static final int kRightJoystick = 1;
    public static final int kButtonBox = 2;


    // Driver Buttons

    // Co-Piolt Button board
    //public static final int kgoToL4Button = 5;

    // Zeroed values, should be in radians
    // source is google document in Electrical for team - module data
    public static final double kDegreesToRadians = Math.PI * 2.0 / 360.0;
    public static final double kZeroedFrontLeft = 295.3 * kDegreesToRadians;    // for FL module 4
    public static final double kZeroedFrontRight = 145.5 * kDegreesToRadians;   // for FR module 1
    public static final double kZeroedBackLeft = 310.9 * kDegreesToRadians;     // for BL module 2
    public static final double kZeroedBackRight = 76.8 * kDegreesToRadians;    // for BR module 3
};