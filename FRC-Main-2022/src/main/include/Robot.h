/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <string>

#include <frc/smartdashboard/SendableChooser.h>

//#include <frc/Joystick.h>
#include <frc/XboxController.h>
#include <frc/TimedRobot.h>
#include <frc/drive/MecanumDrive.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/motorcontrol/PWMSparkMax.h>
#include <frc/motorcontrol/Talon.h>
#include <frc/motorcontrol/MotorControllerGroup.h>
#include <ADIS16470_IMU.h>
#include <iostream>
#include <frc/smartdashboard/SmartDashboard.h>

class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override;
  void RobotPeriodic() override;
  //void AutonomousInit() override;
  //void AutonomousPeriodic() override;
  //void TeleopInit() override;
  void TeleopPeriodic() override;
  //void TestPeriodic() override;

 private:
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
  frc::PWMSparkMax m_frontLeftMotor{2};
  frc::PWMSparkMax m_rearLeftMotor{3};
  frc::PWMSparkMax m_frontRightMotor{0};
  frc::PWMSparkMax m_rearRightMotor{1};
  //frc::PWMSparkMax m_TurretMain}{?}; need to assign id later
  //frc::Talon m_ShooterMain{?}; need to assign id later
  frc::MotorControllerGroup m_leftMotor{m_frontLeftMotor,m_rearLeftMotor};
  frc::MotorControllerGroup m_rightMotor{m_frontRightMotor, m_rearRightMotor};
  frc::DifferentialDrive m_robotDrive{m_leftMotor, m_rightMotor};
  
  frc::XboxController m_driverController{0};
  
  //frc::Joystick m_leftStick{0};
  //frc::Joystick m_rightStick{1};

};
