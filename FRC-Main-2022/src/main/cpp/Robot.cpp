// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include <frc/TimedRobot.h>
/**
 * This is a demo program showing how to use Mecanum control with the
 * MecanumDrive class.
 */
  void Robot::RobotInit()
  {
    // Invert the right side motors. You may need to change or remove this to
    // match your robot.
    m_rearLeft.SetInverted(true);
    m_rearRight.SetInverted(true);
  }

  void Robot::RobotPeriodic()
  {
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
    /* Use the joystick X axis for lateral movement, Y axis for forward
     * movement, and Z axis for rotation.
     */
    m_robotDrive.DriveCartesian(m_stick.GetX(), m_stick.GetY(), m_stick.GetZ());
  }

#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
}
#endif
