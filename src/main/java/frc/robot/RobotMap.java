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
    public static final int kAlgaeArmExtendMotor = 9;
    public static final int kCoralDeliveryRightCoralControlMotor = 22;
    public static final int kCoralDeliveryLeftCoralControlMotor = 23;
    public static final int kCoralDeliveryIndexMotor = 21;

    // Analog Input addresses
    public static final int kFrontLeftAnalogInput = 0;
    public static final int kBackLeftAnalogInput = 1;
    public static final int kBackRightAnalogInput = 2;
    public static final int kFrontRightAnalogInput = 3;

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
    public static final int kLeftLockWheels = 3;
    public static final int kRightResetNavXButton = 10;
    public static final int kLeftTroughButton = 4;
    public static final int kRightTroughButton = 2;
    // public static final int kRightResetRestAbsoluteEncoderButton = null;

    // Co-Piolt Button board
    public static final int kplaceAtL4Button = 11;
    public static final int kplaceAtL3Button = 6;
    public static final int kplaceAtL2Button = 5;
    public static final int kDeliverAlgaeButton = 3;
    public static final int kindexCoralRightButton = 8;
    public static final int kindexCoralLeftButton = 12;
    public static final int kalgaeHighIntake = 4;
    public static final int kalgaeMediumIntake = 2;
    public static final int kdelivercoralButton = 10;
    public static final int kmoveLifttoBase = 9;
    public static final int kwallIntake = 7;

    // Zeroed values, should be in radians
    // source is google document in Electrical for team - module data
    public static final double kDegreesToRadians = Math.PI * 2.0 / 360.0;
    public static final double kZeroedFrontLeft = 178.8 * kDegreesToRadians;    // for FL module 4
    public static final double kZeroedFrontRight = 3.7 * kDegreesToRadians;   // for FR module 3
    public static final double kZeroedBackLeft = 138.9 * kDegreesToRadians;     // for BL module 1
    public static final double kZeroedBackRight = 212.5 * kDegreesToRadians;    // for BR module 2
};