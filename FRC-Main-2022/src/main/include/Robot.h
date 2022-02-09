/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <string>
#include <iostream>

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/smartdashboard/SendableChooser.h>

#include <frc/TimedRobot.h>
#include <frc/drive/DifferentialDrive.h>

//Gyro
#include <cmath>
#include <frc/ADIS16470_IMU.h>

//Color Sensor
#include <frc/util/Color.h>
#include "rev/ColorMatch.h"
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
#include <rev/CANSparkMax.h>
#include <frc/motorcontrol/Talon.h>
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

  //Robot task setup

  //ColorSensorV3

  static constexpr auto i2cPort = frc::I2C::Port::kOnboard;

  rev::ColorSensorV3 m_colorSensor{i2cPort};
  frc::Color detectedColor = m_colorSensor.GetColor();
  double IR = m_colorSensor.GetIR();
  uint32_t proximity = m_colorSensor.GetProximity();
  rev::ColorMatch m_colorMatcher;

 private:

  //Drive-train

  frc::PWMSparkMax m_frontLeftMotor{2};
  frc::PWMSparkMax m_rearLeftMotor{3};
  frc::PWMSparkMax m_frontRightMotor{0};
  frc::PWMSparkMax m_rearRightMotor{1};
  rev::CANSparkMax m_turretMotor{7, rev::CANSparkMax::MotorType::kBrushless};
  //frc::PWMSparkMax m_TurretMain}{?}; need to assign id later
  //frc::Talon m_ShooterRight{?}; need to assign id later
  //frc::Talon m_ShooterLeft{?}; need to assign id later
  frc::MotorControllerGroup m_leftMotor{m_frontLeftMotor,m_rearLeftMotor};
  frc::MotorControllerGroup m_rightMotor{m_frontRightMotor, m_rearRightMotor};
  frc::DifferentialDrive m_robotDrive{m_leftMotor, m_rightMotor};
  frc::XboxController m_driverController{0};

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

};
