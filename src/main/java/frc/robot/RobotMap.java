// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

public final class RobotMap {

    // Swerve Motor Assemblies - SparkMax
    // Drive motor CAN == module #
    // Steer motor CAN == module # + 10
    public static final int kFrontLeftModule = 4;
    public static final int kBackLeftModule = 1;
    public static final int kBackRigvhtModule = 2;
    public static final int kFrontRightModule = 3;

    // CAN motor addresses
        // SparkMax
    public static final int kLiftLeadMotor = 5;
    public static final int kLiftFollowerMotor = 6;
    public static final int kAlgaeClawIntakeMotor = 7;
    public static final int kAlgaeClawShootMotor = 8;
    public static final int kAlgaeArmExtendMotor = 9;
    public static final int kCoralDeliveryControlMotor = 23;
    public static final int kCoralDeliveryIndexMotor = 21;
    public static final int kClimbMotor = 31;

    // Analog Input addresses
    public static final int kFrontLeftAnalogInput = 0;
    public static final int kBackLeftAnalogInput = 1;
    public static final int kBackRightAnalogInput = 2;
    public static final int kFrontRightAnalogInput = 3;

    // REV Pneumatic Hub solenoid addresses

    // Digital IO addresses
    public static final int kCoralDetectInput = 8;

    // Operator input USB ports
    public static final int kLeftJoystick = 0;
    public static final int kRightJoystick = 1;
    public static final int kButtonBox = 2;
    public static final int kXboxTester = 3;

    // Driver Buttons
    public static final int kLeftLockWheels = 4;
    public static final int kRightResetNavXButton = 10;
    public static final int kLeftL4DeliverButton = 10;
    public static final int kLeftAlgaeClawRetractButton = 9;
    public static final int kRightAlignWithReefButton = 2;
    public static final int kLeftExtendClimbButton = 5;
    public static final int kLeftRetractClimbButton = 6;
    public static final int kRightPathfindToReefButton = 5;
    public static final int kRightAlignWithIntakeButton = 3;
    public static final int kLeftDeliverL1Button = 2;

    // Co-Piolt Button board
    public static final int kPlaceAtL4Button = 11;
    public static final int kPlaceAtL3Button = 6;
    public static final int kPlaceAtL2Button = 5;
    public static final int kIndexCoralRightButton = 8;
    public static final int kIndexCoralLeftButton = 12;
    public static final int kDeliverCoralButton = 10;
    public static final int kMoveLiftToBase = 9;
    public static final int kWallIntake = 7;
    public static final int kAlgaeIntakePrep = 4; 
    public static final int kAlgaeIntakeLow = 3;
    public static final int kAlgaeIntakeHigh = 1;
    public static final int kAlgaeIntakeStopAxis = 1;
    public static final int kDeliverAlgaeButton = 2;

    // Zeroed values, should be in radians
    // source is google document in Electrical for team - module data
    public static final double kDegreesToRadians = Math.PI * 2.0 / 360.0;
    public static final double kZeroedFrontLeft = 178.8 * kDegreesToRadians;    // for FL module 4
    public static final double kZeroedFrontRight = 3.7 * kDegreesToRadians;   // for FR module 3
    public static final double kZeroedBackLeft = 138.9 * kDegreesToRadians;     // for BL module 1
    public static final double kZeroedBackRight = 212.5 * kDegreesToRadians;    // for BR module 2
};