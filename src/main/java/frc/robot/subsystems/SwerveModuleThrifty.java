package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkRelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

public class SwerveModuleThrifty {
    private static final double kSteerP = 1.8;
    private static final double kSteerI = 0.0;
    private static final double kSteerD = 0.0;
    private static final double kSteerFF = 0.0;
    private static final double kSteerIZone = 1.0;
    private static final double kSteerMaxVelocity = 0;
    private static final double kSteerMaxAcceleration = 0;

    // measured circumference as 12.375 inches = radius 1.97
    // private static final double kWheelRadius = 1.90 * 0.0254; // 2" * 0.0254 m / inch
    // private static final double kDrivePositionFactor = (2.0 * Math.PI * kWheelRadius * kGearboxRatio);
    private static final double kGearboxRatio = 1.0 / 5.14; // One turn of the wheel is 5.14 turns of the motor, output gear 16T, pinion gear 14T
    private static final double kWheelCircumference = 0.3; // m
    private static final double kInvertedMotorMultiplier = -1.0;
    private static final double kDrivePositionFactor = kInvertedMotorMultiplier * kWheelCircumference * kGearboxRatio;
    private static final int kDriveCurrentLimitAmps = 100;
    private static final int kSteerCurrentLimitAmps = 40; 
    private static final double kSteerMotorRotationsPerRevolution = 25;

    private TalonFX m_driveMotor;
    private VelocityVoltage m_velocityVoltage;
    private SparkMax m_steerMotor;
    private final SparkMaxConfig m_steerConfig;
    private AnalogEncoder m_steerAbsoluteEncoder;
    private SparkRelativeEncoder m_steerEncoder;
    private SparkClosedLoopController m_steerController;
    private final double m_steerOffset;
    private final String m_name;

    private final DoublePublisher m_sparkRelativeAnglePub;
    private final DoublePublisher m_absoluteAnglePub;
    private final DoublePublisher m_encoderRawAbsoluteAnglePub;
    private final DoublePublisher m_krakenPositionPub;
    private final DoublePublisher m_krakenVelocityPub;
    private final DoubleSubscriber m_targetAngleSub;
    private final DoubleSubscriber m_SteerKpSub;
    private final DoubleSubscriber m_SteerKiSub;
    private final DoubleSubscriber m_SteerKdSub;
    private final DoubleSubscriber m_SteerFfSub;
    private final DoubleSubscriber m_SteerMaxVelocitySub;
    private final DoubleSubscriber m_SteerMaxAccelerationSub;
    private final BooleanSubscriber m_updatesteerPIDSub;
    private final BooleanPublisher m_updatesteerPIDPub;

    public SwerveModuleThrifty(int motorAssembly, double steerOffset, int analogInput, String name)
    {
        this(motorAssembly, motorAssembly+ 10, analogInput, steerOffset, name);
    }

    // https://github.com/Team364/BaseFalconSwerve/blob/main/src/main/java/frc/robot/SwerveModule.java
    private SwerveModuleThrifty(int driveMotor, int steerMotor, int steerAbsoluteEncoder, double steerOffset, String name)
    {
        m_name = name;
        m_steerOffset = steerOffset;

        m_driveMotor = new TalonFX(driveMotor);
        m_steerMotor = new SparkMax(steerMotor, SparkMax.MotorType.kBrushless);
        m_velocityVoltage = new VelocityVoltage(0).withSlot(0);

        m_steerAbsoluteEncoder = new AnalogEncoder(steerAbsoluteEncoder);

        var krakenLimitsConfig =  new CurrentLimitsConfigs()
            .withStatorCurrentLimit(kDriveCurrentLimitAmps)
            .withSupplyCurrentLimit(40)
            .withSupplyCurrentLimitEnable(true)
            .withStatorCurrentLimitEnable(true);
        m_driveMotor.getConfigurator().apply(krakenLimitsConfig);

        Slot0Configs slot0Configs = new Slot0Configs();
        slot0Configs.kS = 0.05; // Add 0.05 V output to overcome static friction
        slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        slot0Configs.kP = 0.05; // An error of 1 rps results in 0.05 V output
        slot0Configs.kI = 0; // no output for integrated error
        slot0Configs.kD = 0; // no output for error derivative
        m_driveMotor.getConfigurator().apply(slot0Configs);
        m_driveMotor.getConfigurator().apply(new ClosedLoopRampsConfigs().withVoltageClosedLoopRampPeriod(0.100));
        m_driveMotor.setNeutralMode(NeutralModeValue.Brake);

        m_steerConfig = new SparkMaxConfig();
        m_steerConfig.idleMode(IdleMode.kBrake)
            .smartCurrentLimit(kSteerCurrentLimitAmps)
            .inverted(true)
            .voltageCompensation(12.6);

        m_steerConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pidf(kSteerP, kSteerI, kSteerD, kSteerFF)
            .iZone(kSteerIZone);

        // Enable PID wrap around for the turning motor. This will allow the PID
        // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
        // to 10 degrees will go through 0 rather than the other direction which is a
        // longer route.
        m_steerConfig.closedLoop.positionWrappingEnabled(true);
        m_steerConfig.closedLoop.positionWrappingMinInput(0);
        m_steerConfig.closedLoop.positionWrappingMaxInput(2 * Math.PI);
        m_steerConfig.encoder.positionConversionFactor(2 * Math.PI / kSteerMotorRotationsPerRevolution);

        m_steerMotor.configure(m_steerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        m_steerEncoder = (SparkRelativeEncoder) m_steerMotor.getEncoder();          
        m_steerController = m_steerMotor.getClosedLoopController();

        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        // get the subtable called "datatable"
        NetworkTable datatable = inst.getTable("serveMod1");
        // subscribe to the topic in "datatable" called "target angle"
        // default value is 0
        var targetDoubleTopic = datatable.getDoubleTopic("target angle");
        targetDoubleTopic.publish().set(0);
        m_targetAngleSub = targetDoubleTopic.subscribe(0.0);
        // publish to the topic in "datatable" called "absolute angle"
        m_absoluteAnglePub = datatable.getDoubleTopic(m_name + "/absolute angle with offset").publish();
        m_sparkRelativeAnglePub = datatable.getDoubleTopic(m_name + "/spark relativeangle").publish();
        m_encoderRawAbsoluteAnglePub = datatable.getDoubleTopic(m_name + "/absolute encoder angle").publish();
        m_krakenPositionPub = datatable.getDoubleTopic(m_name + "/kraken position").publish();
        m_krakenVelocityPub = datatable.getDoubleTopic(m_name + "/kraken velocity").publish();
      
        var steerKpTopic = datatable.getDoubleTopic("steer Kp");
        steerKpTopic.publish().set(kSteerP);
        m_SteerKpSub = steerKpTopic.subscribe(kSteerP);
       
        var steerKiTopic = datatable.getDoubleTopic("steer Ki");
        steerKiTopic.publish().set(kSteerI);
        m_SteerKiSub = steerKiTopic.subscribe(kSteerI);
       
        var steerDTopic = datatable.getDoubleTopic("steer D");
        steerDTopic.publish().set(kSteerD);
        m_SteerKdSub = steerDTopic.subscribe(kSteerD);
        
        var steerFfTopic = datatable.getDoubleTopic("steer FF");
        steerFfTopic.publish().set(kSteerFF);
        m_SteerFfSub = steerFfTopic.subscribe(kSteerFF);

        var steerMaxVelocityTopic = datatable.getDoubleTopic("steer Max Velocity");
        steerMaxVelocityTopic.publish().set(kSteerMaxVelocity);
        m_SteerMaxVelocitySub = steerMaxVelocityTopic.subscribe(kSteerMaxVelocity);

        var steerMaxAccelerationTopic = datatable.getDoubleTopic("steer Max Acceleration");
        steerMaxAccelerationTopic.publish().set(kSteerMaxAcceleration);
        m_SteerMaxAccelerationSub = steerMaxAccelerationTopic.subscribe(kSteerMaxAcceleration);
        
        var updatesteerPIDTopic = datatable.getBooleanTopic("update Steer PID");
        m_updatesteerPIDPub = updatesteerPIDTopic.publish();
        m_updatesteerPIDPub.set(false);
        m_updatesteerPIDSub = updatesteerPIDTopic.subscribe(false);
    }

    public void dashboardUpdate() {
        m_krakenPositionPub.set(getDrivePosition());
        m_krakenVelocityPub.set(getDriveVelocity());

        m_encoderRawAbsoluteAnglePub.set(getThriftyEncoder().getDegrees());
        m_absoluteAnglePub.set(getAbsoluteEncoderPosition().getDegrees());
        m_sparkRelativeAnglePub.set(getRelativeEncoderPosition().getDegrees());

        if (m_updatesteerPIDSub.get())
        {
            double steerKp = m_SteerKpSub.get();
            m_steerConfig.closedLoop.pidf(steerKp, m_SteerKiSub.get(), m_SteerKdSub.get(), m_SteerFfSub.get());
            m_steerMotor.configure(m_steerConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

            double target = m_targetAngleSub.get();
            setSteerAngleInRadians(target * 2 * Math.PI / 360.0);
    
            m_updatesteerPIDPub.set(false);
        }
    }

    private Rotation2d getThriftyEncoder()
    {
        // From ThriftyBot User's Guide: 
        // The signal pin is a 12-bit absolute position reference with the lower bound being the ground pin and the
        // upper bound being the 5V pin. This 5V is relative to the 5V rail on the Roborio, which is often not exactly
        // 5V. This allows for a resolution of 1/4096*360 (~.09) degrees.
        double rotations = m_steerAbsoluteEncoder.get();
        double degrees = rotations * 360;
        if (degrees < 0)
          degrees += 360;    

        return Rotation2d.fromDegrees(degrees);
    }

    private Rotation2d getAbsoluteEncoderPosition()
    {
        // convert from absolute Thrifty coder turning counter-clockwise as positive to
        //  relative encoder in assembly turning clockwise as positive

        double startingAngle = m_steerOffset - getThriftyEncoder().getRadians();
    
        if (startingAngle < 0)
        {
          startingAngle = startingAngle + (2 * Math.PI);
        }

        return Rotation2d.fromRadians(startingAngle);
    }

    private Rotation2d getRelativeEncoderPosition()
    {
        return Rotation2d.fromRadians(m_steerEncoder.getPosition());
    }

    public double getDriveVelocity()
    {
        return m_driveMotor.getVelocity().getValueAsDouble() * kDrivePositionFactor;
    }

   public double getDrivePosition()
    {
        return m_driveMotor.getPosition().getValueAsDouble() * kDrivePositionFactor;
    }

   public SwerveModuleState getSwerveModuleState()
    {
        return new SwerveModuleState(getDriveVelocity(), getRelativeEncoderPosition());
    }

   public SwerveModulePosition getPosition()
    {
        return new SwerveModulePosition(getDrivePosition(), getRelativeEncoderPosition());
    }

    /*
     * Sets this modules 
     */
    public void setDesiredState(SwerveModuleState referenceState)
    {
        referenceState.optimize(getRelativeEncoderPosition());
       
        setDriveVelocity(referenceState.speedMetersPerSecond);

        setSteerAngleInRadians(referenceState.angle.getRadians());
    }

    public void setDriveVelocity(double targetSpeed)
    {
        // https://github.com/CrossTheRoadElec/Phoenix6-Examples/blob/main/java/VelocityClosedLoop/src/main/java/frc/robot/Robot.java
        m_driveMotor.setControl(m_velocityVoltage.withVelocity(kInvertedMotorMultiplier * targetSpeed / kDrivePositionFactor));
    }

    public void setSteerAngleInRadians(double targetAngleInRadians)
    {
        SmartDashboard.putNumber(m_name + "/Setpoint", targetAngleInRadians);
        m_steerController.setReference(targetAngleInRadians, SparkMax.ControlType.kPosition);
    }

    /*
     * Resets the SparkMax Alternative Encoder to match the absolute Mag encoder,
     * setting the position of the Mag Encoder to the SparkMax Alternative Encoder 
     */
    public void resetAngleEncoderToAbsolute()
    {
        m_steerEncoder.setPosition(getAbsoluteEncoderPosition().getRadians());
    }

    public void lockWheelAtAngleInDegrees(double degrees)
    {
       // setDriveVelocity(0);
        var angleInRadians = degrees * Math.PI / 180.0;
        setSteerAngleInRadians(angleInRadians);
    }

    public void checkRelativeEncoderToAbsoluteEncoder()
    {
        //we put that the difference should be less than 0.025 which is approximately 0.5% margin of error
        // 0.025 radians is ~ 1.5 degrees
        if (Math.abs(getAbsoluteEncoderPosition().getRadians() - getRelativeEncoderPosition().getRadians()) > 0.025)
        {
            resetAngleEncoderToAbsolute();
        }
    }
}
