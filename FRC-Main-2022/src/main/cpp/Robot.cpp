// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include <frc/TimedRobot.h>

void Robot::RobotInit()
{
  // Assigning Falcons
  m_ShooterLeft = new TalonFX(5);
  m_ShooterRight = new TalonFX(6);

  // We need to invert one side of the drivetrain so that positive voltages
  // result in both sides moving forward. Depending on how your robot's
  // gearbox is constructed, you might have to invert the left side instead.
  m_leftMotor.SetInverted(true);

  //ColorSensorV3
  detectedBallColor = InvalidBall;

  AllianceColor = frc::DriverStation::GetAlliance();
  switch (AllianceColor)
  {
    case frc::DriverStation::Alliance::kBlue:
          frc::SmartDashboard::PutString("AllianceColor", "Blue");
          break;
    case frc::DriverStation::Alliance::kRed:
          frc::SmartDashboard::PutString("AllianceColor", "Red");
          break;
    case frc::DriverStation::Alliance::kInvalid:
          frc::SmartDashboard::PutString("AllianceColor", "?");
          break;
  }

  //modifiable variables
  frc::SmartDashboard::PutNumber("shooterMidSpeed", shooterMidSpeed);

  //Limelight code
  table->PutNumber("ledMode", 3);
  table->PutNumber("camMode", 0);

  //Turret code
  m_turretEncoder.SetPosition(0);
  homingState = manual;
  turretMax = -12.5;
  homingDone = false;
}

void Robot::RobotPeriodic()
{
  
  //ColorSensorV3
  frc::Color detectedColor = m_colorSensor.GetColor();
  double IR = m_colorSensor.GetIR();
  uint32_t proximity = m_colorSensor.GetProximity();

  frc::SmartDashboard::PutNumber("ColorSensor.Color...Red", detectedColor.red);
  frc::SmartDashboard::PutNumber("ColorSensor.Color..Green", detectedColor.green);
  frc::SmartDashboard::PutNumber("ColorSensor.Color.Blue", detectedColor.blue);
  frc::SmartDashboard::PutNumber("ColorSensor.IR", IR);
  frc::SmartDashboard::PutNumber("ColorSensor.Proximity", proximity);

  //sets RGB thresh-hold for red and blue ball detection
  if(detectedColor.red >= 0.29){
    detectedBallColor = RedBall;
  }
  else if(detectedColor.blue >= 0.30){ //plz work
    detectedBallColor = BlueBall;
  }
  else{
    detectedBallColor = InvalidBall;
  }

  if(proximity >= 420){
    ballIntaken = true;
  }
  else{
    ballIntaken = false;
  }

  //outputs ^ to driver-station
  switch (detectedBallColor)
  {
    case BlueBall:
          frc::SmartDashboard::PutString("Detected Ball Color", "Blue");
          break;
    case RedBall:
          frc::SmartDashboard::PutString("Detected Ball Color", "Red");
          break;
    case InvalidBall:
          frc::SmartDashboard::PutString("Detected Ball Color", "?");
          break;
  }

  frc::SmartDashboard::PutBoolean("ballIntaken", ballIntaken);

  //sets CorrectBall to either true or false depending on wether or not our ballcolor matches alliance color, ie. red && and red || blue && blue
  if(detectedBallColor == BlueBall && AllianceColor == frc::DriverStation::Alliance::kBlue){
    CorrectBall = true;
  }
  else if(detectedBallColor == RedBall && AllianceColor == frc::DriverStation::Alliance::kRed){
    CorrectBall = true;
  }
  else{
    CorrectBall = false;
  }

  //outputs ^ to driver-station
  frc::SmartDashboard::PutBoolean("Correct Color Ball", CorrectBall);

  //Encoders
  frc::SmartDashboard::PutNumber("Turret Encoder Position", m_turretEncoder.GetPosition());
  frc::SmartDashboard::PutNumber("Climber Encoder Position", m_climberEncoder.GetPosition());

  frc::SmartDashboard::PutNumber("off set left third addition", offsetLeftShooterSpeedAddition);
  frc::SmartDashboard::PutNumber("off set center third addition", offsetCenterShooterSpeedAddition);
  frc::SmartDashboard::PutNumber("off set right third addition", offsetRightShooterSpeedAddition);

  frc::SmartDashboard::PutNumber("off set left turret addition", offsetLeftTurretAimAddition);
  frc::SmartDashboard::PutNumber("off set center turret addition", offsetCenterTurretAimAddition);
  frc::SmartDashboard::PutNumber("off set right turret addition", offsetRightTurretAimAddition);

  //Limit switch
  frc::SmartDashboard::PutBoolean("Turret Limit Switch Position", m_turretlimitSwitch.Get());

  //Shooter
  shooterRightOutput = m_ShooterRight->GetSelectedSensorVelocity();

  frc::SmartDashboard::PutNumber("Right Shooter Velocity", shooterRightOutput);
  frc::SmartDashboard::PutNumber("Shooter Target Speed", shooterTargetSpeed);

  frc::SmartDashboard::PutNumber("intake delay timer", m_tim3r.Get().value());
  frc::SmartDashboard::PutBoolean("ball delay", ballDelayed);
  frc::SmartDashboard::PutBoolean("ball chambered", ballChambered);
  frc::SmartDashboard::PutNumber("intake position", m_intakeEncoder.GetPosition());

  frc::SmartDashboard::PutNumber("off-set addition", offsetAdditionX);
  frc::SmartDashboard::PutNumber("off-set multipier", offsetMultiplyX);

  frc::SmartDashboard::PutNumber("turret current speec", m_turretMotor.Get());

  //auto
  frc::SmartDashboard::PutNumber("Time 1", m_time.Get().value());
  frc::SmartDashboard::PutNumber("Time 1", m_timeToo.Get().value());

  //Homing mode
  JLeftZ = JLeft.GetZ() * 15;

  frc::SmartDashboard::PutNumber("JLeft Z", JLeftZ);

  switch (homingState)
  {
  case automatic:
    frc::SmartDashboard::PutString("Homing State", "Automatic");
      break;
  case manual:
    frc::SmartDashboard::PutString("Homing State", "Manual");
      break;
  case homingOff:
    frc::SmartDashboard::PutString("Homing State", "Off");
      break;
  default:
    break;
  }

  frc::SmartDashboard::PutNumber("shooterAutoSpeedCurrent", shooterAutoSpeedCurrent);

  shooterMidSpeed = frc::SmartDashboard::GetNumber("shooterMidSpeed", shooterMidSpeed);

  //limelight table data
  targetDetect = table->GetNumber("tv",0.0);
  targetX = table->GetNumber("tx",0.0);
  targetY = table->GetNumber("ty",0.0) + 25.0;
  targetYTrue = table->GetNumber("ty",0.0);

  frc::SmartDashboard::PutNumber("Limelight y +25", targetY);

}

void Robot::AutonomousInit(){

  frontRightMax = 42;
  rearLeftMax = 42 * -1; 

  m_frontRightEncoder.SetPosition(0);
  m_rearLeftEncoder.SetPosition(0);

  m_time.Stop();
  m_time.Reset();
  m_timeToo.Stop();
  m_timeToo.Reset();

  autoDriven = false;

  intakeTargetSpeed = 0;
}

void Robot::AutonomousPeriodic(){
  m_timeToo.Start();
      
  m_ShooterRight->Set(ControlMode::Velocity, shooterAutonomousSpeed);
  m_ShooterLeft->Set(ControlMode::Velocity, shooterAutonomousSpeed * -1);

  shooterLeftOutput = m_ShooterLeft->GetMotorOutputPercent();
  shooterRightOutput = m_ShooterRight->GetMotorOutputPercent();

  if(m_timeToo.Get().value() >= 3){
    m_intake.Set(intakeFastSpeed);
  }

  if(m_frontRightEncoder.GetPosition() < frontRightMax && m_rearLeftEncoder.GetPosition() > rearLeftMax){
    m_leftMotor.Set(0.1);
    m_rightMotor.Set(0.1);
    autoDriven = true;
  }
  else{
    m_leftMotor.Set(0);
    m_rightMotor.Set(0);

    if(autoDriven == true){
      m_time.Start();
    }
  }

  if(m_time.Get().value() >= 4){
  
    if(m_turretlimitSwitch.Get()){
      m_turretMotor.Set(0);
      homingDone = true;
      m_turretEncoder.SetPosition(0);
      homingState = homingOff;
    }
    else{
      m_turretMotor.Set(0.1);
      //m_intake.Set(0);
      intakeMode = manualIntake;
    }
      m_ShooterRight->Set(ControlMode::Velocity, 0);
      m_ShooterLeft->Set(ControlMode::Velocity, 0);
      intakeMode = manualIntake;
  }
}

void Robot::TeleopInit(){
  m_time.Stop();
  m_time.Reset();

  intakeMode = autoIntake;
}
  
void Robot::TeleopPeriodic(){

  GetYRegion();

  //Turret Code

  double LeftTrigger = buttonBoard.GetRawButton(3);
  double RightTrigger = buttonBoard.GetRawButton(8);

  double TurretSpeed = -1 * TriggerSpeed * LeftTrigger + RightTrigger * TriggerSpeed; // makes sure if you try to turn the turret both left and right, it will cancel out and stop

  turretPosition = m_turretEncoder.GetPosition();

  if(homingDone == true && buttonBoard.GetRawButtonPressed(4)){
    if(readyToShoot == false){
      homingState = automatic;
      intakeMode = autoIntake;
      shooterMode = stopSpeed;
    }
    else{
      homingState = automatic;
      intakeMode = intakeFire;
      shooterMode = autoSpeed;
      m_tim3r.Stop();
      m_tim3r.Reset();
      m_timer4.Stop();
      m_timer4.Reset();
      m_timeToo.Stop();
      m_timeToo.Reset();
      timerStarted = false;
    }
  }
  
  if(buttonBoard.GetRawButtonPressed(3) || buttonBoard.GetRawButtonPressed(8) || buttonBoard.GetRawButtonPressed(11)){
    if(homingDone){
      homingState = homingOff;
    }
    else{
      homingState = manual;
    }
  }

  switch (homingState)
  {
  case automatic:

  shooterMidSpeed = frc::SmartDashboard::GetNumber("shooterMidSpeed", shooterMidSpeed);

    if(turretTargetSpeed <= -0.12){
      turretTargetSpeed = -0.12;
    }
    else if(turretTargetSpeed >= 0.12){
      turretTargetSpeed = 0.12;
    }

    turretOffset = frc::SmartDashboard::GetNumber("Turret Offset", 0);

    turretTargetSpeed = TriggerSpeed * ((targetX + turretOffset /*GetShooterOffset()*/) / 19.0);
  
    if(m_turretEncoder.GetPosition() < turretMax){ //makes sure once if the limit switch for the turret is ever tripped the encoder get sets back to 0
      m_turretMotor.Set(0);
    }
    else if(m_turretlimitSwitch.Get()){
      m_turretMotor.Set(0);
    }
    else{
      m_turretMotor.Set(turretTickBack);
    }

    break;
  case manual:
    if(TurretSpeed > 0){
      m_turretMotor.Set(TurretSpeed);
    }
    else{
      m_turretMotor.Set(0);
    }
    break;
  case homingOff:
    if(TurretSpeed < 0 && turretPosition > turretMax){
      m_turretMotor.Set(TurretSpeed);
    } 
    else if(TurretSpeed > 0 && turretPosition < 0){
      m_turretMotor.Set(TurretSpeed);
    }
    else{
      m_turretMotor.Set(0);
    }
    break;
  default:
    break;
  }
  if(m_turretlimitSwitch.Get()){
    m_turretEncoder.SetPosition(0);
  }

  m_robotDrive.TankDrive(-JLeft.GetY()*0.85,-JRight.GetY()*0.85);

    //Shooter & intake Code 
    if(JLeft.GetRawButtonPressed(8)){
    shooterTargetSpeed -= 100;
    }
    if(JLeft.GetRawButtonPressed(9)){
      shooterTargetSpeed += 100;
    }

    if(buttonBoard.GetRawButton(11)){
      //shooterMidSpeed = shooterTargetSpeed;
      shooterTargetSpeed = 0.0;
      intakeTargetSpeed = 0.0; 
      shooterAlive = false;
      intakeMode = manualIntake;
      shooterMode = manualSpeed;
    }
    else if(buttonBoard.GetRawButton(2)){
      shooterTargetSpeed = shooterMidSpeed;
      shooterAlive = true;
      shooterMode = manualSpeed;
    }

    if(buttonBoard.GetRawButtonPressed(7)){
      intakeMode = manualIntake;
      intakeTargetSpeed = intakeFeedSpeed /*intakeFastSpeed*/;
    }
    else if(buttonBoard.GetRawButtonPressed(9)){
      intakeMode = manualIntake;
      intakeTargetSpeed = intakeFastSpeed * -1.0;
    }

    if(detectedBallColor == InvalidBall){
      m_intake.Set(intakeTargetSpeed);
    }
    else{
      m_intake.Set(0.0);
    }
 
    Climber();
    
  //Homing code
  if(homingState == manual){

    if(m_turretlimitSwitch.Get()){
      m_turretEncoder.SetPosition(0);
      homingState = homingOff;
      homingDone = true;
    }
  }

  switch (intakeMode)
  {
  case autoIntake:  //manual control during auto intake will reset values for autoIntake
    if(ballDelayed){
        if(m_intakeEncoder.GetPosition() >= intakeBackup){
          readyToShoot = true;
          intakeTargetSpeed = 0.0;
      }
    }
    else if(ballChambered){ //reverses the intake for auto-intake mode
      if(m_tim3r.Get().value() >= 0.5){ //plz work
        ballDelayed = true;
        intakeBackup = m_intakeEncoder.GetPosition() + 6;
        intakeTargetSpeed = intakeReverse;
      }
    }
    else if(ballIntaken){
      ballChambered = true;
      intakeTargetSpeed = 0.0;
      m_tim3r.Stop(); //plz work
      m_tim3r.Reset();
      m_tim3r.Start();

      m_timer4.Stop(); //plz work
      m_timer4.Reset();
      m_timer4.Start();
    }
    else if(ballIntaken == false){
      intakeTargetSpeed = intakeFastSpeed;
    }
    break;
  case manualIntake:
    ballChambered = false; //do we have a ball chambered in the intake
    readyToShoot = false;
    ballDelayed = false;
    break;
  case intakeFire:
    //targetYValid = (targetY > yTable[4]);
    //GetYRegion();

    targetXValid = (targetX > -targetMaxX && targetX < targetMaxX);
    if(timerStarted){
      intakeTargetSpeed = intakeFeedSpeed;
      if(m_timeToo.Get().value() > intakeFireDelay){
        intakeMode = autoIntake;
        ballChambered = false;
        readyToShoot = false;
        ballDelayed = false;
      }
    }
    else if(m_tim3r.Get().value() > 1.5 && targetDetect == 1.000 && targetYValid && targetXValid){
      intakeTargetSpeed = intakeFastSpeed;
      if(!timerStarted){
        m_timeToo.Reset();
        m_timeToo.Start();
        timerStarted = true;
      }
    }
    break;
  default:
    break;
  }

  m_intake.Set(intakeTargetSpeed); //sets the speed based on any assignments further up

  switch (shooterMode)
  {
  case stopSpeed:
    shooterTargetSpeed = 0;
    break;
  case manualSpeed:
    break;
  case autoSpeed:
    
    shooterTargetSpeed = GetShooterSpeed();

    m_tim3r.Start();
    m_timer4.Start();
    if(m_timeToo.Get().value() > intakeFireDelay){
      shooterMode = stopSpeed;
    }
    break;
  default:
    break;
  }

  //sets the speed based on any assignments further up

  m_ShooterLeft->Set(ControlMode::Velocity, shooterTargetSpeed * -1);
  m_ShooterRight->Set(ControlMode::Velocity, shooterTargetSpeed);
}

void Robot::TestPeriodic(){
  
  double targetX = table->GetNumber("tx",0.0);
  bool targetDetect = table->GetBoolean("tv",false);

  double visionSpeed = TriggerSpeed * (targetX / 30.0);

  frc::SmartDashboard::PutNumber("Vision Horizontal Offset", targetX);
  frc::SmartDashboard::PutNumber("Turret Speed for Vision", visionSpeed);

  m_turretMotor.Set(visionSpeed);

}

Robot::yRegion Robot::GetYRegion(){
  //limelight table data

  targetDetect = (table->GetNumber("tv",0.0) == 1.0);
  targetY = table->GetNumber("ty",0.0) + 25.0;

  if(!targetDetect)
  {
    targetYValid = false;
    return notDetected;
  }

  if(targetY >= yTable[0]){
    frc::SmartDashboard::PutString("Y Region", "yTooClose");
    targetYValid = true;
    return yTooClose;
  }
  else if(targetY >= yTable[1]){
    frc::SmartDashboard::PutString("Y Region", "yClose");
    targetYValid = true;
    return yClose;
  }
  else if(targetY >= yTable[2]){
    frc::SmartDashboard::PutString("Y Region", "yMid");
    targetYValid = true;
    return yMid;
  }
  else if(targetY >= yTable[3]){
    frc::SmartDashboard::PutString("Y Region", "yFar");
    targetYValid = true;
    return yFar;
  }
  else if(targetY < yTable[3]){
    frc::SmartDashboard::PutString("Y Region", "yTooFar");
    targetYValid = false;
    return yTooFar;
  }
  frc::SmartDashboard::PutBoolean("Y Valid", targetYValid);
}
Robot::shooterRegion Robot::GetShooterRegion(){
  //shooter regions
  if(m_turretEncoder.GetPosition() > -4.25){
    frc::SmartDashboard::PutString("shooterRegion", "Right");
    return shooterRight;
  }
  else if(m_turretEncoder.GetPosition() < -4.25 && m_turretEncoder.GetPosition() > -8.5){
    frc::SmartDashboard::PutString("shooterRegion", "Center");
    return shooterCenter;
  }
  else{
    frc::SmartDashboard::PutString("shooterRegion", "Left");
    return shooterLeft;
  }
}
double Robot::GetShooterSpeed(){
  shooterRegion  shooterRegionForSpeed = GetShooterRegion();
  yRegion yRegionForSpeed = GetYRegion();
  
  switch (yRegionForSpeed)
  {
  case yTooClose:
  case yClose:
  case yMid:
  case yFar:
  return speedTable[(int)yRegionForSpeed][(int)shooterRegionForSpeed];
    break;
  
  default:
    return shooterTargetSpeed;
    break;
  }
}
double Robot::GetShooterOffset(){
  shooterRegion  shooterRegionForSpeed = GetShooterRegion();
  yRegion yRegionForSpeed = GetYRegion();
  
  switch (yRegionForSpeed)
  {
  case yTooClose:
  case yClose:
  case yMid:
  case yFar:
  return shooterOffsetTable[(int)yRegionForSpeed][(int)shooterRegionForSpeed];
    break;
  case yTooFar:
    shooterTargetSpeed = tooFarSpeed;
  default:
    return 0;
    break;
  }
}
void Robot::Climber(){
  // Climber winch code

  if(buttonBoard.GetRawButton(1)){
        m_climberWinch.Set(-1.0);
  }
  else if(buttonBoard.GetRawButton(6)){
        m_climberWinch.Set(1.0);
  }
  else{
    m_climberWinch.Set(0.0);
  }
}

//Destructor (Cleans up stuff)

Robot::~Robot(){
 delete m_ShooterLeft;
 delete m_ShooterRight;
}


#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
}
#endif
