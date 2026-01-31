package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
    private final SparkFlex shooterLeader;
    private final SparkFlex shooterFollower;
    private final SparkMax hoodMotor;

    private final SparkClosedLoopController shooterClosedLoop;

    private double targetHoodAngle = 0.0;
    private double targetRPM = 0.0;

    private final NetworkTable nt = NetworkTableInstance.getDefault().getTable("shooter");



    public ShooterSubsystem() {
        shooterLeader = new SparkFlex(ShooterConstants.SHOOTER_LEADER_PORT, MotorType.kBrushless);
        shooterFollower = new SparkFlex(ShooterConstants.SHOOTER_FOLLOWER_PORT, MotorType.kBrushless);

        SparkFlexConfig shooterLeaderConfig = new SparkFlexConfig();
        shooterLeaderConfig.idleMode(IdleMode.kCoast).inverted(ShooterConstants.SHOOTER_LEADER_INVERTED);

        shooterLeaderConfig.closedLoop.pid(ShooterConstants.SHOOTER_P, ShooterConstants.SHOOTER_I, ShooterConstants.SHOOTER_D);

        SparkFlexConfig shooterFollowerConfig = new SparkFlexConfig();
        shooterFollowerConfig.idleMode(IdleMode.kCoast).inverted(ShooterConstants.SHOOTER_FOLLOWER_INVERTED)
                .follow(ShooterConstants.SHOOTER_LEADER_PORT);

        SparkMaxConfig hoodConfig = new SparkMaxConfig();
        hoodConfig.idleMode(IdleMode.kBrake).inverted(ShooterConstants.HOOD_INVERTED);
        hoodConfig.absoluteEncoder.positionConversionFactor(ShooterConstants.HOOD_GEAR_RATIO);


        // Configure motors
        shooterLeader.configure(shooterLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        shooterFollower.configure(shooterFollowerConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        shooterClosedLoop = shooterLeader.getClosedLoopController();

        hoodMotor = new SparkMax(ShooterConstants.HOOD_MOTOR_PORT, MotorType.kBrushless);
    }

    @Override
    public void periodic() {

        nt.putValue("target_hood_angle", NetworkTableValue.makeDouble(targetHoodAngle));
        nt.putValue("target_shooter_rpm", NetworkTableValue.makeDouble(targetRPM));
        nt.putValue("hood_angle", NetworkTableValue.makeDouble(shooterLeader.getEncoder().getVelocity()));
        nt.putValue("shooter_rpm", NetworkTableValue.makeDouble(getHoodAngle()));


        targetHoodAngle = MathUtil.clamp(targetHoodAngle, ShooterConstants.HOOD_MIN_ANGLE, ShooterConstants.HOOD_MAX_ANGLE);

        shooterClosedLoop.setSetpoint(targetRPM, ControlType.kVelocity);
        ShooterConstants.HOOD_PID.calculate(targetHoodAngle, getHoodAngle());
        

    }

    public double getHoodAngle() {
        return Units.rotationsToDegrees(hoodMotor.getAbsoluteEncoder().getPosition());
    }

    public void setShooterRPM(double rpm) {
        targetRPM = MathUtil.clamp(rpm, 0.0, ShooterConstants.SHOOTER_MAX_RPM);
    }

    public double getShooterRPM() {
        return !RobotBase.isSimulation() ? shooterLeader.getEncoder().getVelocity() : targetRPM;
    }


}
