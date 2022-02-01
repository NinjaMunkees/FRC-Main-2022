/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <string>

#include <frc/smartdashboard/SendableChooser.h>

#include <frc/XboxController.h>
#include <frc/TimedRobot.h>
#include <frc/drive/MecanumDrive.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/motorcontrol/PWMSparkMax.h>
#include <frc/motorcontrol/Talon.h>
#include <frc/motorcontrol/MotorControllerGroup.h>
#include <frc/ADIS16470_IMU.h>
#include <iostream>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/Encoder.h>
#include <frc/CAN.h>

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

  frc::PWMSparkMax m_frontLeftMotor{2};
  frc::PWMSparkMax m_rearLeftMotor{3};
  frc::PWMSparkMax m_frontRightMotor{0};
  frc::PWMSparkMax m_rearRightMotor{1};
  //frc::PWMSparkMax m_TurretMain}{?}; need to assign id later
  //frc::Talon m_ShooterRight{?}; need to assign id later
  //frc::Talon m_ShooterLeft{?}; need to assign id later
  frc::MotorControllerGroup m_leftMotor{m_frontLeftMotor,m_rearLeftMotor};
  frc::MotorControllerGroup m_rightMotor{m_frontRightMotor, m_rearRightMotor};
  frc::DifferentialDrive m_robotDrive{m_leftMotor, m_rightMotor};
  frc::XboxController m_driverController{0};
  
};
