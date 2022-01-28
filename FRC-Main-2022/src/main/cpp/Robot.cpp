// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include <frc/TimedRobot.h>

  void Robot::RobotInit()
  {

  }

  void Robot::RobotPeriodic()
  {
     // Code that handles the adis 16470 imu raw data
    frc::SmartDashboard::PutNumber("YawAngle", m_imu.GetAngle());
    frc::SmartDashboard::PutNumber("XCompAngle", m_imu.GetXComplementaryAngle());
    frc::SmartDashboard::PutNumber("YCompAngle", m_imu.GetYComplementaryAngle());
    m_setDecRate = frc::SmartDashboard::GetBoolean("SetDecRate", false);
    m_decRate = frc::SmartDashboard::GetNumber("DecRate", 4);
    m_runCal = frc::SmartDashboard::GetBoolean("RunCal", false);
    m_configCal = frc::SmartDashboard::GetBoolean("ConfigCal", false);
    m_reset = frc::SmartDashboard::GetBoolean("Reset", false);
    m_setYawAxis = frc::SmartDashboard::GetBoolean("SetYawAxis", false);
    m_yawSelected = m_yawChooser.GetSelected();
  }

  void Robot::TeleopPeriodic()
  {
    // Drive with tank style
    m_robotDrive.TankDrive(m_leftStick.GetY(), m_rightStick.GetY());
  }

#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
}
#endif
