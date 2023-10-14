// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
 
import java.io.ObjectInputStream.GetField;
import java.util.List;
import java.util.function.BiConsumer;
import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.Pigeon2Configuration;
import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.fasterxml.jackson.core.type.ResolvedType;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.commands.PPRamseteCommand;
import com.pathplanner.lib.server.PathPlannerServer;
import com.pathplanner.lib.server.PathPlannerServerThread;

import SimulationMath.WheelVelocityCalculator;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.simulation.ADXRS450_GyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.math.util.Units;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController; 

public class Drivetrain extends SubsystemBase {
  
  public WPI_TalonFX right1 = new WPI_TalonFX(1);
  public WPI_TalonFX right2 = new WPI_TalonFX(2);
  public WPI_TalonFX left1 = new WPI_TalonFX(3);
  public WPI_TalonFX left2 = new WPI_TalonFX(4);

  MotorControllerGroup leftMotors = new MotorControllerGroup(left1, left2);
  MotorControllerGroup rightMotors = new MotorControllerGroup(right1, right2);

  DifferentialDrive drive = new DifferentialDrive(leftMotors, rightMotors);
  DifferentialDriveKinematics kinematics;
  DifferentialDriveOdometry m_odometry;

  DifferentialDrivetrainSim m_drivetrainSimulator;
  WheelVelocityCalculator leftVelocity, rightVelocity;

  Field2d m_field;
  ADXRS450_Gyro gyro = new ADXRS450_Gyro();
  ADXRS450_GyroSim m_gyroSim = new ADXRS450_GyroSim(gyro);
  
  /** Creates a new Drivetrain. */
  public Drivetrain() {
 

    LinearSystem<N2, N2, N2> kDrivetrainPlant =
        LinearSystemId.identifyDrivetrainSystem(
            1.98,
            0.2,
            1.5,
            0.3);

    m_drivetrainSimulator =
    new DifferentialDrivetrainSim(
      kDrivetrainPlant,
      DCMotor.getFalcon500(2),
      8.45,
      Units.inchesToMeters(32),
      6 / 2.0,
        VecBuilder.fill(0, 0, 0, 0.1, 0.1, 0.005, 0.005));
    
    
    m_field = new Field2d();
    
    kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(23.5));
    m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()), getLeftEncoderMeters(), getRightEncoderMeters(), new Pose2d(0, 0, new Rotation2d()));
    leftVelocity = new WheelVelocityCalculator(left1);
    rightVelocity = new WheelVelocityCalculator(right1);

    SmartDashboard.putData("Field", m_field);
  }

  public void arcadeDrive(double fwd, double rot) {
    drive.arcadeDrive(fwd, rot*0.6);
  }

  double leftRate, rightRate;
  @Override
  public void periodic() {
    m_drivetrainSimulator.setInputs(
      left1.get() * RobotController.getBatteryVoltage(),
      right1.get() * RobotController.getBatteryVoltage());

    left1.setSelectedSensorPosition((int) (m_drivetrainSimulator.getLeftPositionMeters() / (2 * Math.PI * Units.inchesToMeters(3)) * (2048 * 8.45)));
    right1.setSelectedSensorPosition((int) (m_drivetrainSimulator.getRightPositionMeters() / (2 * Math.PI * Units.inchesToMeters(3)) * (2048 * 8.45)));
 

    m_gyroSim.setAngle(-m_drivetrainSimulator.getHeading().getDegrees());

    m_drivetrainSimulator.update(0.020);
    SmartDashboard.updateValues();  

  }

  @Override
    public void simulationPeriodic() {

    m_odometry.update(
      Rotation2d.fromDegrees(getHeading()),
      getLeftEncoderMeters(),
      getRightEncoderMeters());
    m_field.setRobotPose(getFieldPos());
    SmartDashboard.putData("Field", m_field);
    SmartDashboard.putNumber("Left Velocity", leftVelocity.calculateWheelVelocity());
    SmartDashboard.putNumber("Right Velocity", rightVelocity.calculateWheelVelocity());
  }

  BiConsumer<Double, Double>  driveVolts = (left, right) -> {
    leftMotors.setVoltage(left);
    rightMotors.setVoltage(right);
    drive.feed();
  };


  public void driveVoltsTank(double left, double right){
    leftMotors.setVoltage(left*12);
    rightMotors.setVoltage(right*12);
    drive.feed();
  }

  Supplier<Pose2d> getPose = () -> {
    return m_odometry.getPoseMeters();
  };
  
  public Pose2d getFieldPos(){
    return m_odometry.getPoseMeters();
    }

  Supplier<DifferentialDriveWheelSpeeds> getCurrentSpeeds = () -> {
    return new DifferentialDriveWheelSpeeds(m_drivetrainSimulator.getLeftVelocityMetersPerSecond(), m_drivetrainSimulator.getRightVelocityMetersPerSecond());
  };
  
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(Rotation2d.fromDegrees(getHeading()), getLeftEncoderMeters(), getRightEncoderMeters(), pose);
  }

  public void resetPose() {
    m_odometry.resetPosition(Rotation2d.fromDegrees(getHeading()), getLeftEncoderMeters(), getRightEncoderMeters(), new Pose2d());
  }
  
  public void zeroGyro(){
    gyro.reset();
    System.out.println(Rotation2d.fromDegrees(getHeading()));
  }

  public Rotation2d getRobotAngle2d() {
    return Rotation2d.fromDegrees(getHeading());
  }

  public double getHeading() {
    return Math.IEEEremainder(gyro.getAngle(), 360) * (-1);
  }

  // public double getLeftEncoderVelocity () {
  //   return -(2 * Math.PI * Units.inchesToMeters(3) * ((left1.getSelectedSensorVelocity() / 2048) / 8.45) * 10);
  // }

  public double getLeftEncoderMeters () {
    return m_drivetrainSimulator.getLeftPositionMeters();
  }


  // public double getRightEncoderVelocity () {
  //   // return (right1.getSelectedSensorVelocity() / 2048) * 2 * Math.PI * Units.inchesToMeters(6);
  //   return (2 * Math.PI * Units.inchesToMeters(3) * ((right1.getSelectedSensorVelocity() / 2048) / 8.45) * 10);
  // }

  public double getRightEncoderMeters () {
    return m_drivetrainSimulator.getRightPositionMeters();
  }

  public CommandBase resetEncoders() {
    return runOnce(() -> {
      right1.setSelectedSensorPosition(0);
      right2.setSelectedSensorPosition(0);
      left1.setSelectedSensorPosition(0);
      left2.setSelectedSensorPosition(0);
    });
  }
 
  private void updateState(List<State> list) {
    PathPlannerServer.sendActivePath(list); 
  }

  public double getDrawnCurrentAmps() {
    return m_drivetrainSimulator.getCurrentDrawAmps();
  }

  public Command followTrajectoryCommand(String traj) {
    PathPlannerTrajectory AutoPath = PathPlanner.loadPath(traj, new PathConstraints(3 , 3));
    PathPlannerServer.sendActivePath(AutoPath.getStates()); 

    this.resetOdometry(AutoPath.getInitialPose());
    
    return new PPRamseteCommand(
          AutoPath,
          getPose,
          new RamseteController(2, 0.7),
          new SimpleMotorFeedforward(1, 1),
          kinematics,
          getCurrentSpeeds,
          new PIDController(3, 0, 0.1),
          new PIDController(3, 0, 0.1),
          driveVolts,
          false,
          this
          ).raceWith(new RepeatCommand(new RunCommand(() -> updateState(AutoPath.getStates()))));
  }

 

  

}
