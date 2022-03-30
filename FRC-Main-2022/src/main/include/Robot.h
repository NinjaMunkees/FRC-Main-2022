/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <string>
#include <iostream>

#include <chrono>
#include <thread>

#include <frc/DriverStation.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/DigitalInput.h>

#include <frc/TimedRobot.h>
#include <frc/drive/DifferentialDrive.h>

//Color Sensor
#include "rev/ColorSensorV3.h"

//Basic Vision
#include <cstdio>
#include "cameraserver/CameraServer.h"

//Limelight
#include "wpi/span.h"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableValue.h"
#include "networktables/NetworkTableInstance.h"

//Controllers
#include <frc/Joystick.h>
#include <frc/XboxController.h>

//Motor controllers, CAN, and encoder
#include <frc/CAN.h>
#include <frc/Encoder.h>
#include "ctre/Phoenix.h"
#include <rev/CANSparkMax.h>
#include <frc/motorcontrol/PWMSparkMax.h>
#include <frc/motorcontrol/MotorControllerGroup.h>


class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override; 
  void AutonomousPeriodic() override;
  void TeleopInit() override;      
  void TeleopPeriodic() override;
  void TestPeriodic() override;
  ~Robot();

 private:

  //Funcions
  void Turret();
  void Shooter();
  void Climber();
  //void Grip();
  void Intake();

  //Motors

  rev::CANSparkMax m_turretMotor{7, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_frontRightMotor{1, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_rearRightMotor{2, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_frontLeftMotor{3, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_rearLeftMotor{4, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_intake{8, rev::CANSparkMax::MotorType::kBrushless};
  //rev::CANSparkMax m_climberGrip{10, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_climberWinch{9, rev::CANSparkMax::MotorType::kBrushless};
  TalonFX * m_ShooterLeft;
  TalonFX * m_ShooterRight;
  frc::MotorControllerGroup m_leftMotor{m_frontLeftMotor,m_rearLeftMotor};
  frc::MotorControllerGroup m_rightMotor{m_frontRightMotor, m_rearRightMotor};
  frc::DifferentialDrive m_robotDrive{m_leftMotor, m_rightMotor};
  frc::XboxController m_driverController{0};
  frc::Joystick JLeft{0};
  frc::Joystick JRight{1};
  frc::Joystick buttonBoard{2};

  //Encoders

  rev::SparkMaxRelativeEncoder m_frontRightEncoder = m_frontRightMotor.GetEncoder();
  rev::SparkMaxRelativeEncoder m_rearRightEncoder = m_rearRightMotor.GetEncoder();
  rev::SparkMaxRelativeEncoder m_frontLeftEncoder = m_frontLeftMotor.GetEncoder();
  rev::SparkMaxRelativeEncoder m_rearLeftEncoder = m_rearLeftMotor.GetEncoder();

  rev::SparkMaxRelativeEncoder m_intakeEncoder = m_intake.GetEncoder();
  //rev::SparkMaxRelativeEncoder m_gripEncoder = m_climberGrip.GetEncoder();
  rev::SparkMaxRelativeEncoder m_turretEncoder = m_turretMotor.GetEncoder();
  rev::SparkMaxRelativeEncoder m_climberEncoder = m_climberWinch.GetEncoder();

  //Grip (possibly deprecated)

  double gripStartPosition; //start position for grip
  double gripPosition; //current grip position
  enum gripState{gripOpening, gripClosing, gripStopped};
  gripState gripState; //^
  double gripMax; //max grip position
  double gripSpeed; //used in calculating grip ramp-up and ramp-down

  //Turret

  double turretStartPosition; //maybe deprecated
  double turretPosition; //current turret encoder position
  enum homingState{automatic, manual, homingOff}; //auto-aim, manual homing (can't rotate turret counter clock-wise untill limit switch has been tripped(turn the turret fully clock-wise)), homing finished(limits put in place)
  homingState homingState; //^
  double turretMax; //max position for turret (from limit switch), starts at 0 and goes to approx. -13

  //Auto

  double frontRightMax;
  double rearRightMax;
  double frontLeftMax;
  double rearLeftMax;

  //speeds for shooter motors, changes based on other variables

  double shooterLeftOutput;
  double shooterRightOutput;

  frc::Timer m_time; //timer
  frc::Timer m_timeToo; //timer

  bool autoDriven; //has auto been run, this is a requirement to home in auto

  //Limit switch

  frc::DigitalInput m_turretlimitSwitch {0};  //limit switch for turret

  //Shooter & Intake
  
  double shooterTargetSpeed = 2000; //current shooter speed, changes based on other variables
  double shooterMidSpeed = 2000; //used when no auto adjusting speed
  double shooterFastSpeed = 22000; //currently un-used
  double shooterTestSpeed = 1000; // if we want to test in the work shop, ie. auto-intake mode
  const double shooterAutonomousSpeed = 5600; //probably wrong number
  const double shooterSlowSpeed = 5300; //currently un-used
  double intakeBackup; //controls how much to backup the intake in auto-intake mode
  double intakeTargetSpeed = -0.8; //current intake speed
  const double intakeFastSpeed = -0.8; //speed for intake
  const double intakeFeedSpeed = -0.15; //speed for feeding ball in auto aim
  const double intakeReverse = 0.8; //reverse speed for intake
  const double TriggerSpeed = 0.1; //used to cancel out turret movement when attempting to move the turret both left and right
  bool shooterAlive;
  bool intakeAlive;
  bool ballChambered = false; //do we have a ball in the intake, used in ayto intake mode
  bool ballDelayed = false;
  double JLeftZ; //throttle on left flight-stick
  frc::Timer m_tim3r; //
  
  double autoIntakeStopShooter = 2.0; //stops the shooter once we have shot a ball in auto intake & fire

  double shooterLowGoal = 3500; //ideal speed for low goal
  double shooterAutoSpeedCurrent; //current speed for auto aim based on x and y values
  double targetMinY = 7.0; //minimum limelight y value for auto shooting
  double targetMaxY = 29.0; //maximum limelight y value for auto shooting
  double targetMaxX = 5.0; //maximum limelight x value (+ or -) for auto shooting CHANGE LATER
  bool readyToShoot = false; // done intaking and ready to shoot
  enum intakeMode{autoIntake, manualIntake, intakeFire};  //intake one ball, user control, fire once ball has been chambered
  intakeMode intakeMode; // ^
  enum shooterMode{manualSpeed, autoSpeed, stopSpeed}; //run at set speed, speed controlled by y value, 0
  shooterMode shooterMode; //^
  bool timerStarted = false; 
  double intakeFireDelay = 2.0                 ; //delay for firing the ball in auto intake mode

  //off-sets for auto aim (horizontal)
  double offsetAdditionX = -10.0; //currently un-used
  double offsetMultiplyX = -5.0; //currently un-used
  double offsetLeftTurretAimAddition;
  double offsetCenterTurretAimAddition;
  double offsetRightTurretAimAddition;


  //offsets for auto speed based on turret location

  double offsetLeftShooterSpeedAddition = 100; //offset for left-most third of turret rotation
  double offsetCenterShooterSpeedAddition = 100; //offset for center-most third of turret rotation
  double offsetRightShooterSpeedAddition = 100; //offset for right-most third of turret rotation
  double offsetAdditionY;

  //shooter regions
  enum shooterRegion{shooterRight, shooterCenter, shooterLeft}; //which third of the turrets range of motion are we currently in
  shooterRegion shooterRegion;//^

  //ColorSensorV3

  static constexpr auto i2cPort = frc::I2C::Port::kOnboard;

  rev::ColorSensorV3 m_colorSensor{i2cPort}; //color sensor

  frc::DriverStation::Alliance AllianceColor; //which alliance are we on, red or blue

  enum BallColor{RedBall, BlueBall, InvalidBall}; //which color ball do we have in our intake
  BallColor detectedBallColor;//^
  bool CorrectBall = false; //does our currently chambered ball match our alliance color

  //Limelight code 

  std::shared_ptr<nt::NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");

  double targetX; //current x offset for limelight, range approx. -30 to 30
  double targetY; //current y offset for limelight
  bool targetXValid; //valid x range for limelight, what zone horizontally should score
  bool targetYValid; //valid y range for limeligt, how close can we shoot from
  bool targetDetect; //do we see a valid target
  double turretTargetSpeed; //speed for the shooter, calculated off of x value

  //limit switch code

  bool homingDone;
  double turretEncoderStart;

};
