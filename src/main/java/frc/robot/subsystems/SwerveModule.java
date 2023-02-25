// Packaging subsystems
package frc.robot.subsystems;

// Imports
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utility.CtreUtils;
import frc.robot.utility.RevUtils;

import static frc.robot.Constants.Swerve.Module.*;
import static frc.robot.Constants.Swerve.kMaxSpeedMetersPerSecond;

//Defines class under Subsystem
public class SwerveModule extends SubsystemBase {

    //PID Slot
    private final int POS_SLOT = 0;
    private final int VEL_SLOT = 1;

    //Defining variables for motors/encoders
    int m_moduleNumber;
    CANSparkMax m_turnMotor;
    WPI_TalonFX m_driveMotor;
    private SparkMaxPIDController m_turnController;
    private final RelativeEncoder m_turnEncoder;
    DutyCycleEncoder m_angleEncoder;

    //Defining the offset for the encoders
    double m_angleOffset;

    //Defining variables for current and last angles
    double m_currentAngle;
    double m_lastAngle;

    //Defining 2D Pose for Field Oriented
    Pose2d m_pose;

    //Defines Feed Forward for Motors
    SimpleMotorFeedforward feedforward =
            new SimpleMotorFeedforward(
                    ksDriveVoltSecondsPerMeter,
                    kaDriveVoltSecondsSquaredPerMeter,
                    kvDriveVoltSecondsSquaredPerMeter);

    //Defines PID Controller
    private final ProfiledPIDController m_turningPIDController
            = new ProfiledPIDController(1, 0, 0,
            new TrapezoidProfile.Constraints(2 * Math.PI, 2 * Math.PI));

    //Defines what goes into a swerve module
    public SwerveModule(
            int moduleNumber,
            CANSparkMax turnMotor,
            WPI_TalonFX driveMotor,
            DutyCycleEncoder angleEncoder,
            double angleOffset) {
        //Turns original variables into variables that can be used here
        m_moduleNumber = moduleNumber;
        m_turnMotor = turnMotor;
        m_driveMotor = driveMotor;
        m_angleOffset = angleOffset;
        
        //Reset drive configs to the default
        m_driveMotor.configFactoryDefault();

        //Configs persistent settings
        m_driveMotor.configAllSettings(CtreUtils.generateDriveMotorConfig());
        
        //Reset turn configs to the default
        m_turnMotor.restoreFactoryDefaults();

        //Sets PID, frame period, and current limits
        RevUtils.setTurnMotorConfig(m_turnMotor);

        //Sets Idle mode to brake instead of coast
        m_turnMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        
        m_angleEncoder = angleEncoder;
        
        //Gets encoder value from turn motor
        m_turnEncoder = m_turnMotor.getEncoder();

        //Multiplies conversion factor by output to give position/velocity
        m_turnEncoder.setPositionConversionFactor(kTurnRotationsToDegrees);
        m_turnEncoder.setVelocityConversionFactor(kTurnRotationsToDegrees / 60);

        //incorporates pid into turn motor
        m_turnController = m_turnMotor.getPIDController();

        //resets the angle to 0 when initialized
        resetAngleToAbsolute();
    }

    //gets module # of swerve module
    public int getModuleNumber() {
        return m_moduleNumber;
    }

    //resets angle to 0 when initialized
    public void resetAngleToAbsolute() {
        //sets "0 angle"
        double angle = 0 - pwmToDegrees(m_angleEncoder.getAbsolutePosition());

        //physically sets position to 0
        m_turnEncoder.setPosition(angle);
    }

    //gets current angle
    public double getHeadingDegrees() {
        return m_currentAngle;
    }

    //gets current position using polar coords
    public Rotation2d getHeadingRotation2d() {
        return Rotation2d.fromDegrees(getHeadingDegrees());
    }

    //gets how far has been driven
    public double getDriveMeters() {
        return m_driveMotor.getSelectedSensorPosition() * kDriveDistancePerPulse;
    }

    //gets current velocity
    public double getDriveMetersPerSecond() {
        return m_driveMotor.getSelectedSensorVelocity() * kDriveDistancePerPulse * 10;
    }

    //sets method on how to bring turn motor to desired position
    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        //sets current angle as position of encoder
        m_currentAngle = m_turnEncoder.getPosition();

        //sets desired state taking into account optimizations (Ex. 45 degrees over 225 degrees)
        desiredState = RevUtils.optimize(desiredState, getHeadingRotation2d());

        //sets desired velocity
        double velocity = desiredState.speedMetersPerSecond / (kDriveDistancePerPulse * 10);
        
        //sets drive mode and demands for drive motors
        m_driveMotor.set(
                ControlMode.Velocity,
                velocity,
                DemandType.ArbitraryFeedForward,
                feedforward.calculate(desiredState.speedMetersPerSecond));

        //sets desired angle
        double angle =
                (Math.abs(desiredState.speedMetersPerSecond) <= (kMaxSpeedMetersPerSecond * 0.01))
                        ? m_lastAngle
                        : desiredState.angle.getDegrees() - m_angleOffset; // Prevent rotating module if speed is less than 1%. Prevents Jittering.
        //sets pid reference
        m_turnController.setReference(angle, CANSparkMax.ControlType.kPosition, POS_SLOT);
        m_lastAngle = angle;
    }

    //gets current velocity and position
    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveMetersPerSecond(), getHeadingRotation2d());
    }

    //gets exact position
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDriveMeters(), getHeadingRotation2d());
    }

    // sets 2D Pose
    public void setModulePose(Pose2d pose) {
        m_pose = pose;
    }

    //gets current 2D Pose
    public Pose2d getModulePose() {
        return m_pose;
    }

    //puts current headings and angles into Smart Dashboard
    private void updateSmartDashboard() {
        SmartDashboard.putNumber(
                "module " + m_moduleNumber + " heading", getState().angle.getDegrees());
        SmartDashboard.putNumber(
                "module " + m_moduleNumber + " CANCoder reading", pwmToDegrees(m_angleEncoder.getAbsolutePosition()));
    }

    //changes pwm values into degrees
    private double pwmToDegrees(double signal) {
        return signal * 360.0;
    }

    //updates smart dashboard periodically
    @Override
    public void periodic() {
        updateSmartDashboard();
    }
}
