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
    public static final int kBackLeftModule = 1;
    public static final int kBackRigvhtModule = 2;
    public static final int kFrontRightModule = 3;
    // unused module - 5

    // CAN motor addresses
        // SparkMax
    public static final int kLiftLeadMotor = 5;
    public static final int kLiftFollowerMotor = 6;
    public static final int kAlgaeClawIntakeMotor = 8;
    public static final int kAlgaeClawShootMotor = 7;
    public static final int kCoralDeliveryRightCoralControlMotor = 22;
    public static final int kCoralDeliveryLeftCoralControlMotor = 23;
    public static final int kCoralDeliveryIndexMotor = 21;

    // Analog Input addresses
    

    // REV Pneumatic Hub solenoid addresses
    public static final int kAlgaeClawSolenoid = 1;

    // Digital IO addresses
    public static final int kCoralDetectInput = 1;
    public static final int kLiftDetectInput = 2;

    // Operator input USB ports
    public static final int kLeftJoystick = 0;
    public static final int kRightJoystick = 1;
    public static final int kButtonBox = 2;
    public static final int kXboxTester = 3;

    // Driver Buttons

    // Co-Piolt Button board
    public static final int kplaceAtL4Button = 5;
    public static final int kplaceAtL3Button = 9;
    public static final int kplaceAtL2Button = 10
    ;
    public static final int kintakeAlgaeButton = 6;
    public static final int kleftTroughPlaceButton = 7;
    public static final int krightTroughPlaceButton = 8;

    // Zeroed values, should be in radians
    // source is google document in Electrical for team - module data
    public static final double kDegreesToRadians = Math.PI * 2.0 / 360.0;
    public static final double kZeroedFrontLeft = 179.2 * kDegreesToRadians;    // for FL module 4
    public static final double kZeroedFrontRight = 4.4 * kDegreesToRadians;   // for FR module 3
    public static final double kZeroedBackLeft = 139.2 * kDegreesToRadians;     // for BL module 1
    public static final double kZeroedBackRight = 213.6 * kDegreesToRadians;    // for BR module 2
};