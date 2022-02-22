/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <string>
#include <iostream>

#include <frc/DriverStation.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/smartdashboard/SendableChooser.h>

#include <frc/TimedRobot.h>
#include <frc/drive/DifferentialDrive.h>

//Gyro
#include <cmath>
#include <frc/ADIS16470_IMU.h>

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
  //void AutonomousInit() override;
  //void AutonomousPeriodic() override;
  void TeleopInit() override;      
  void TeleopPeriodic() override;
  //void TestPeriodic() override;
  ~Robot();

  //Robot task setup

 private:

  //Drive-train

  //frc::PWMSparkMax m_frontLeftMotor{2};
  //frc::PWMSparkMax m_rearLeftMotor{3};
  //frc::PWMSparkMax m_frontRightMotor{0};
  //frc::PWMSparkMax m_rearRightMotor{1};
  rev::CANSparkMax m_turretMotor{7, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_frontRightMotor{1, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_rearRightMotor{2, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_frontLeftMotor{3, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_rearLeftMotor{4, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_intake{8, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_climberGrip{10, rev::CANSparkMax::MotorType::kBrushless};
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

  rev::SparkMaxRelativeEncoder m_gripEncoder = m_climberGrip.GetEncoder();
  rev::SparkMaxRelativeEncoder m_turretEncoder = m_turretMotor.GetEncoder();
  rev::SparkMaxRelativeEncoder m_climberEncoder = m_climberWinch.GetEncoder();

  //Shooter
  
  double shooterTargetSpeed = 0.8;
  double shooterSlowSpeed = 5;

  //Gyro

  frc::SendableChooser<std::string> m_autoChooser;
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom = "My Auto";
  std::string m_autoSelected;
  frc::ADIS16470_IMU m_imu{};
  frc::SendableChooser<std::string> m_yawChooser;
  const std::string kYawDefault = "Z-Axis";
  const std::string kYawXAxis = "X-Axis";
  const std::string kYawYAxis = "Y-Axis";
  std::string m_yawSelected;
  bool m_runCal = false;
  bool m_configCal = false;
  bool m_reset = false;
  bool m_setYawAxis = false;
  uint16_t m_decRate = 4;
  bool m_setDecRate = false;
  frc::ADIS16470_IMU::IMUAxis m_yawActiveAxis = frc::ADIS16470_IMU::IMUAxis::kZ;

  //ColorSensorV3

  static constexpr auto i2cPort = frc::I2C::Port::kOnboard;

  rev::ColorSensorV3 m_colorSensor{i2cPort};

  frc::DriverStation::Alliance AllianceColor;

  enum BallColor{RedBall, BlueBall, InvalidBall};

  //Limelight code 

  std::shared_ptr<nt::NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
  double targetOffsetAngle_Horizontal = table->GetNumber("tx",0.0);
  double targetOffsetAngle_Vertical = table->GetNumber("ty",0.0);
  double targetArea = table->GetNumber("ta",0.0);
  double targetSkew = table->GetNumber("ts",0.0);

};
