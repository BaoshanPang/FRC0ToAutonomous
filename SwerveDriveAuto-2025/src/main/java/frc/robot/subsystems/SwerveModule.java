package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule {

    private final SparkMax driveMotor;
    private final SparkMax turningMotor;

//    private final RelativeEncoder driveEncoder;
    //private final RelativeEncoder turningEncoder;

//    private final PIDController turningPidController;

    private final AnalogInput absoluteEncoder;
    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffsetRad;


    public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed,
            int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed) {

        SparkMaxConfig config1 = new SparkMaxConfig();
        this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
        this.absoluteEncoderReversed = absoluteEncoderReversed;
        absoluteEncoder = new AnalogInput(absoluteEncoderId);

        driveMotor = new SparkMax(driveMotorId, MotorType.kBrushless);
        //driveMotor.setInverted(driveMotorReversed);
        config1.inverted(driveMotorReversed);
        //driveEncoder = driveMotor.getEncoder();
        //driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
        //driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);
        config1.encoder.positionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter)
                .velocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);
        config1.closedLoop.pid(ModuleConstants.kPTurning, 0, 0);   
        driveMotor.configure(config1, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        turningMotor = new SparkMax(turningMotorId, MotorType.kBrushless);
        //turningMotor.setInverted(turningMotorReversed);
        SparkMaxConfig config2 = new SparkMaxConfig();
        config2.inverted(turningMotorReversed); 
       // turningEncoder = turningMotor.getEncoder();
       //  turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad);
       // turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec);
        config2.encoder.positionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad)
                .velocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec);
       // turningPidController = new PIDController(ModuleConstants.kPTurning, 0, 0);
        //turningPidController.enableContinuousInput(-Math.PI, Math.PI);
        config2.closedLoop.pid(ModuleConstants.kPTurning, 0, 0);
        turningMotor.configure(config2, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        resetEncoders();
    }

    public double getDrivePosition() {
//        return driveEncoder.getPosition();
        return driveMotor.getEncoder().getPosition();
    }

    public double getTurningPosition() {
//        return turningEncoder.getPosition();
    return turningMotor.getEncoder().getPosition();
    }

    public double getDriveVelocity() {
//        return driveEncoder.getVelocity();
        return driveMotor.getEncoder().getVelocity();
    }

    public double getTurningVelocity() {
//        return turningEncoder.getVelocity();
        return turningMotor.getEncoder().getVelocity();
    }

    public double getAbsoluteEncoderRad() {
        double angle = absoluteEncoder.getVoltage() / RobotController.getVoltage5V();
        angle *= 2.0 * Math.PI;
        angle -= absoluteEncoderOffsetRad;
        return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
    }

    public void resetEncoders() {
//        driveEncoder.setPosition(0);
driveMotor.getEncoder().setPosition(0);
       // turningEncoder.setPosition(getAbsoluteEncoderRad());
        turningMotor.getEncoder().setPosition(getAbsoluteEncoderRad());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    public void setDesiredState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
        state.optimize(getState().angle);
        driveMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
//        turningMotor.set(turningPidController.calculate(getTurningPosition(), state.angle.getRadians()));
        turningMotor.set(state.angle.getRadians());
        SmartDashboard.putString("Swerve[" + absoluteEncoder.getChannel() + "] state", state.toString());
    }

    public void stop() {
       driveMotor.set(0);
       turningMotor.set(0);
    }
}
