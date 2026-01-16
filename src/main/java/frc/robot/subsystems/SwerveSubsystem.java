package frc.robot.subsystems;


import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.SwerveDriveConfig;
import yams.mechanisms.config.SwerveModuleConfig;
import yams.mechanisms.swerve.SwerveDrive;
import yams.mechanisms.swerve.SwerveModule;
import yams.mechanisms.swerve.utility.SwerveInputStream;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.local.SparkWrapper;
import frc.robot.Constants;
import frc.robot.SwerveConstants.*;
import frc.robot.SwerveConstants.SwerveModules.*;

public class SwerveSubsystem extends SubsystemBase
{
  private AngularVelocity          maximumChassisSpeedsAngularVelocity = DegreesPerSecond.of(720);
  private LinearVelocity           maximumChassisSpeedsLinearVelocity  = MetersPerSecond.of(4);

  private final SwerveDrive drive;
  private final Field2d field = new Field2d();

   /**
   * Get a {@link Supplier<ChassisSpeeds>} for the robot relative chassis speeds based on "standard" swerve drive
   * controls.
   *
   * @param translationXScalar Translation in the X direction from [-1,1]
   * @param translationYScalar Translation in the Y direction from [-1,1]
   * @param rotationScalar     Rotation speed from [-1,1]
   * @return {@link Supplier<ChassisSpeeds>} for the robot relative chassis speeds.
   */
  public SwerveInputStream getChassisSpeedsSupplier(DoubleSupplier translationXScalar,DoubleSupplier translationYScalar,DoubleSupplier rotationScalar) {
    return new SwerveInputStream(drive, translationXScalar, translationYScalar, rotationScalar)
      .withMaximumAngularVelocity(maximumChassisSpeedsAngularVelocity)
      .withMaximumLinearVelocity(maximumChassisSpeedsLinearVelocity)
      .withDeadband(Constants.OperatorConstants.DEADBAND)
      .withCubeRotationControllerAxis()
      .withCubeTranslationControllerAxis()
      .withAllianceRelativeControl();
  }

  /**
   * Get a {@link Supplier<ChassisSpeeds>} for the robot relative chassis speeds based on "standard" swerve drive
   * controls.
   *
   * @param translationXScalar Translation in the X direction from [-1,1]
   * @param translationYScalar Translation in the Y direction from [-1,1]
   * @param rotationScalar     Rotation speed from [-1,1]
   * @return {@link Supplier<ChassisSpeeds>} for the robot relative chassis speeds.
   */
  public Supplier<ChassisSpeeds> getSimpleChassisSpeeds(DoubleSupplier translationXScalar, DoubleSupplier translationYScalar, DoubleSupplier rotationScalar) {
    return () -> new ChassisSpeeds(
      maximumChassisSpeedsLinearVelocity.times(translationXScalar.getAsDouble()).in(MetersPerSecond),
      maximumChassisSpeedsLinearVelocity.times(translationYScalar.getAsDouble()).in(MetersPerSecond),
      maximumChassisSpeedsAngularVelocity.times(rotationScalar.getAsDouble()).in(RadiansPerSecond)
    );
  }


  public SwerveModule createModule(SparkMax drive, SparkMax azimuth, CANcoder absoluteEncoder, String moduleName, Translation2d location) {
    MechanismGearing driveGearing   = new MechanismGearing(ModuleConstants.kDriveGearRatio);
    MechanismGearing azimuthGearing = new MechanismGearing(ModuleConstants.kSteerGearRatio);

    SmartMotorControllerConfig driveCfg = new SmartMotorControllerConfig(this)
        .withWheelDiameter(Inches.of(ModuleConstants.kWheelDiameterInches))
        .withClosedLoopController(50, 0, 4)
        .withGearing(driveGearing)
        .withStatorCurrentLimit(Amps.of(ModuleConstants.kDriveCurrentLimit))
        .withTelemetry("driveMotor", SmartMotorControllerConfig.TelemetryVerbosity.HIGH);

    SmartMotorControllerConfig azimuthCfg = new SmartMotorControllerConfig(this)
        .withClosedLoopController(50, 0, 4)
        .withContinuousWrapping(Radians.of(-Math.PI), Radians.of(Math.PI))
        .withGearing(azimuthGearing)
        .withStatorCurrentLimit(Amps.of(ModuleConstants.kSteerCurrentLimit))
        .withTelemetry("angleMotor", SmartMotorControllerConfig.TelemetryVerbosity.HIGH);

    SmartMotorController driveSMC   = new SparkWrapper(drive, DCMotor.getNEO(1), driveCfg);
    SmartMotorController azimuthSMC = new SparkWrapper(azimuth, DCMotor.getNEO(1), azimuthCfg);

    SwerveModuleConfig moduleConfig = new SwerveModuleConfig(driveSMC, azimuthSMC)
        .withAbsoluteEncoder(absoluteEncoder.getAbsolutePosition().asSupplier())
        .withTelemetry(moduleName, SmartMotorControllerConfig.TelemetryVerbosity.HIGH)
        .withLocation(location)
        .withOptimization(true);

    return new SwerveModule(moduleConfig);
  }

  public SwerveSubsystem() {
    Pigeon2 gyro = new Pigeon2(14);

    var fl = createModule(
      new SparkMax(FrontLeft.kDriveMotorID, MotorType.kBrushless),
      new SparkMax(FrontLeft.kSteerMotorID, MotorType.kBrushless),
      new CANcoder(FrontLeft.kAbsoluteEncoderID),
      "frontleft",
      FrontLeft.kModuleLocation
    );

    var fr = createModule(
      new SparkMax(FrontRight.kDriveMotorID, MotorType.kBrushless),
      new SparkMax(FrontRight.kSteerMotorID, MotorType.kBrushless),
      new CANcoder(FrontRight.kAbsoluteEncoderID),
      "frontright",
      FrontRight.kModuleLocation
    );

    var bl = createModule(
      new SparkMax(BackLeft.kDriveMotorID, MotorType.kBrushless),
      new SparkMax(BackLeft.kSteerMotorID, MotorType.kBrushless),
      new CANcoder(BackLeft.kAbsoluteEncoderID),
      "backleft",
      BackLeft.kModuleLocation
    );

    var br = createModule(
      new SparkMax(BackRight.kDriveMotorID, MotorType.kBrushless),
      new SparkMax(BackRight.kSteerMotorID, MotorType.kBrushless),
      new CANcoder(BackRight.kAbsoluteEncoderID),
      "backright",
      BackRight.kModuleLocation
    );

    SwerveDriveConfig config = new SwerveDriveConfig(this, fl, fr, bl, br)
        .withGyro(gyro.getYaw().asSupplier())
        .withStartingPose(new Pose2d(0, 0, Rotation2d.fromDegrees(0)))
        .withTranslationController(new PIDController(1, 0, 0))
        .withRotationController(new PIDController(1, 0, 0));

    drive = new SwerveDrive(config);

    
      setupPathPlanner();
    
    SmartDashboard.putData("Field", field);
  }
  private void setupPathPlanner() {
    RobotConfig config;
    try {
      config = RobotConfig.fromGUISettings();

      AutoBuilder.configure(
          drive::getPose,
          // Robot pose supplier
          drive::resetOdometry,
          // Method to reset odometry (will be called if your auto has a starting pose)
          drive::getRobotRelativeSpeed,
          // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
          (speedsRobotRelative, moduleFeedForwards) -> {
            drive.setRobotRelativeChassisSpeeds(speedsRobotRelative);
          },
          // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
          new PPHolonomicDriveController(
              // PPHolonomicController is the built in path following controller for holonomic drive trains
              new PIDConstants(5.0, 0.0, 0.0),
              // Translation PID constants
              new PIDConstants(5.0, 0.0, 0.0)
              // Rotation PID constants
          ),
          config,
          // The robot configuration
          () -> {
            // Boolean supplier that controls when the path will be mirrored for the red alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

            var alliance = DriverStation.getAlliance();
            return alliance.filter(value -> value == DriverStation.Alliance.Red).isPresent();
          },
          this
          // Reference to this subsystem to set requirements
      );
    } catch (Exception e) {
      e.printStackTrace();
    }


    CommandScheduler.getInstance().schedule(PathfindingCommand.warmupCommand());
  }

  /**
   * Drive the {@link SwerveDrive} object with robot relative chassis speeds.
   *
   * @param speedsSupplier Robot relative {@link ChassisSpeeds}.
   * @return {@link Command} to run the drive.
   */
  public Command drive(Supplier<ChassisSpeeds> speedsSupplier)
  {
    return drive.drive(speedsSupplier);
  }

  public Command setRobotRelativeChassisSpeeds(ChassisSpeeds speeds)
  {
    return run(() -> drive.setRobotRelativeChassisSpeeds(speeds));
  }

  public Command driveToPose(Pose2d pose)
  {
    return drive.driveToPose(pose);
  }

  public Command driveRobotRelative(Supplier<ChassisSpeeds> speedsSupplier)
  {
    return drive.drive(speedsSupplier);
  }

  public Command lock()
  {
    return run(drive::lockPose);
  }

  @Override
  public void periodic()
  {
    drive.updateTelemetry();
    field.setRobotPose(drive.getPose());
  }

  @Override
  public void simulationPeriodic()
  {
    drive.simIterate();
  }
}