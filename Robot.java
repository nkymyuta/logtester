/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
//import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
//import edu.wpi.first.wpilibj.SendableBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
//import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.DigitalInput;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.motorcontrol.SensorCollection;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.ControlMode;

public class Robot extends TimedRobot {
 
  private Joystick m_stick;
  private XboxController m_xbox;
  //private JoystickButton m_button;
  
  WPI_TalonSRX m_Talon;
 
  WPI_TalonSRX s_TalonLeft, s_TalonRight;
  
  WPI_VictorSPX i_Intakefront, i_Intakeback, i_Intakeroller;

  
  SensorCollection m_TalonEncoder;
  SensorCollection s_TalonEncoderLeft;
  SensorCollection s_TalonEncoderRight;
  SensorCollection MaxUpSwitch, MaxDownSwitch;

  Encoder m_Encoder;
  DigitalInput BallSensorFront, BallSensorBack;

  double SetSpeedPoint;
  //double SetShooterPointGet;
  //double SetIntakePointGet;

  //定数宣言
  public static final double SetShooterMagni = 20000;
  public static final double SetCanonMagni = 0.01;
  
  public static final double SetShooterSpeed_P  = 0.35;
  public static final double SetShooterOutSpeed_P  = 1.0;

  public static final double SetIntakeBeltSpeed_P  = 1.0;
  public static final double SetIntakeSpeed_P  = 0.4;
  public static final double SetIntakeOutSpeed_P  = 0.8;




  public int button = 3;

  PidGain pidgain;

  @Override
  public void robotInit() {
    
//-------------------------------------------------------------------------------------

   //コントローラー宣言
    m_stick = new Joystick(0);
    m_xbox  = new XboxController(1);

    m_Talon = new WPI_TalonSRX(3);
    m_TalonEncoder = new SensorCollection(m_Talon);
    MaxUpSwitch    = new SensorCollection(m_Talon);
    MaxDownSwitch  = new SensorCollection(m_Talon);


    s_TalonLeft  = new WPI_TalonSRX(5);
    s_TalonRight = new WPI_TalonSRX(4);
    s_TalonEncoderLeft = new SensorCollection(s_TalonLeft);
    s_TalonEncoderRight = new SensorCollection(s_TalonRight);

    i_Intakefront  = new WPI_VictorSPX(11);
    i_Intakeback   = new WPI_VictorSPX(15);
    i_Intakeroller = new WPI_VictorSPX(14);

    BallSensorFront = new DigitalInput(0);
    BallSensorBack = new DigitalInput(1);

    s_TalonLeft.configFactoryDefault();
    s_TalonRight.configFactoryDefault();

    //PID設定
    //pidgain.s_TalonLeftPIDSet(s_TalonLeft);
    //pidgain.s_TalonRightPIDSet(s_TalonRight);

//-------------------------------------------------------------------------------------

        //s_TalonLeft.configOpenloopRamp(0.1);
        s_TalonLeft.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 
                                                 PidGain.kPIDLoopIdx, PidGain.kTimeoutMs);

        s_TalonLeft.setSensorPhase(true);
        //s_TalonLeft.setInverted(false);

        s_TalonLeft.configNominalOutputForward(0,PidGain.kTimeoutMs);
        s_TalonLeft.configNominalOutputReverse(0,PidGain.kTimeoutMs);
        s_TalonLeft.configPeakOutputForward(1.0,PidGain.kTimeoutMs);
        s_TalonLeft.configPeakOutputReverse(-1.0,PidGain.kTimeoutMs);

        s_TalonLeft.config_kF(PidGain.kPIDLoopIdx, PidGain.kF, PidGain.kTimeoutMs);
        s_TalonLeft.config_kP(PidGain.kPIDLoopIdx, PidGain.kP, PidGain.kTimeoutMs);
        s_TalonLeft.config_kI(PidGain.kPIDLoopIdx, PidGain.kI, PidGain.kTimeoutMs);
        s_TalonLeft.config_kD(PidGain.kPIDLoopIdx, PidGain.kD, PidGain.kTimeoutMs);

//--------------------------------------------------------------------------------------
        
        //s_TalonRight.configOpenloopRamp(0.1);
        s_TalonRight.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 
                                                  PidGain.kPIDLoopIdx, PidGain.kTimeoutMs);

        s_TalonRight.setSensorPhase(true);
        //s_TalonRight.setInverted(false);

        s_TalonRight.configNominalOutputForward(0,PidGain.kTimeoutMs);
        s_TalonRight.configNominalOutputReverse(0,PidGain.kTimeoutMs);
        s_TalonRight.configPeakOutputForward(1.0,PidGain.kTimeoutMs);
        s_TalonRight.configPeakOutputReverse(-1.0,PidGain.kTimeoutMs);

        s_TalonRight.config_kF(PidGain.kPIDLoopIdx, PidGain.kF, PidGain.kTimeoutMs);
        s_TalonRight.config_kP(PidGain.kPIDLoopIdx, PidGain.kP, PidGain.kTimeoutMs);
        s_TalonRight.config_kI(PidGain.kPIDLoopIdx, PidGain.kI, PidGain.kTimeoutMs);
        s_TalonRight.config_kD(PidGain.kPIDLoopIdx, PidGain.kD, PidGain.kTimeoutMs);

//--------------------------------------------------------------------------------------
        
  }


  @Override
  public void robotPeriodic() {


  }

  
  @Override
  public void autonomousInit() {
   
  }

  
  @Override
  public void autonomousPeriodic() {
    

  }

  @Override
  public void teleopInit(){

  }

  @Override
  public void disabledInit(){


  }
  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    //SetShooterPointGet = m_xbox.getY(Hand.kLeft) * SetShooterMagni;

    //--------------------------------------------------------------------------------------

    SmartDashboard.putBoolean("MaxDownSwitchClosed", MaxDownSwitch.isFwdLimitSwitchClosed());
    SmartDashboard.putBoolean("MaxUpSwitchClosed",   MaxUpSwitch.isRevLimitSwitchClosed());

    SmartDashboard.putBoolean("BallSensor",BallSensorFront.get());
    SmartDashboard.putBoolean("BallSensor", BallSensorBack.get());

    SmartDashboard.putBoolean("GetBumper/L",m_xbox.getBumper(Hand.kLeft));
    SmartDashboard.putBoolean("GetBumper/R",m_xbox.getBumper(Hand.kRight));


    SmartDashboard.putNumber("Xboxstick/X/L",m_xbox.getX(Hand.kLeft));
    SmartDashboard.putNumber("Xboxstick/Y/L",m_xbox.getY(Hand.kLeft));
    SmartDashboard.putNumber("Xboxstick/X/R",m_xbox.getX(Hand.kRight));
    SmartDashboard.putNumber("Xboxstick/Y/R",m_xbox.getY(Hand.kRight));


    SmartDashboard.putNumber("TalonEncoder_P",m_TalonEncoder.getAnalogInRaw());
    SmartDashboard.putNumber("TalonEncoder_V",m_TalonEncoder.getAnalogInVel());
    
    SmartDashboard.putNumber("ShootEncoder/L_V",s_TalonEncoderLeft.getAnalogInVel());
    SmartDashboard.putNumber("ShootEncoder/R_V",s_TalonEncoderRight.getAnalogInVel());
 
    SmartDashboard.putNumber("SetshooterPoint",SetSpeedPoint);

    //--------------------------------------------------------------------------------------

    
    //シューターPointセット 
    if(m_xbox.getTriggerAxis(Hand.kRight) > 0.2){
      SetSpeedPoint = m_xbox.getTriggerAxis(Hand.kRight)  *SetShooterMagni;
    }else{
      SetSpeedPoint = 0;
    }


    //インテイク
    if(m_xbox.getTriggerAxis(Hand.kRight) <= 0.2){

      
      if(m_xbox.getBumper(Hand.kLeft)){
        i_Intakefront.set(-SetIntakeBeltSpeed_P);
        i_Intakeroller.set(-SetIntakeSpeed_P);

        s_TalonLeft.set(ControlMode.PercentOutput, SetShooterSpeed_P);
        s_TalonRight.set(ControlMode.PercentOutput, -SetShooterSpeed_P);

       //センサーが二つとも入力してる時(0)だけバック回す
        if(!BallSensorFront.get() && !BallSensorBack.get()){        
          i_Intakeback.set(-SetIntakeBeltSpeed_P);
        }else{        
          i_Intakeback.set(0);
        }

      }else if(m_xbox.getBumper(Hand.kRight)){
        //装填
        i_Intakefront.set(SetIntakeBeltSpeed_P);
        i_Intakeback.set(SetIntakeBeltSpeed_P);
        i_Intakeroller.set(SetIntakeSpeed_P);

        //s_TalonLeft.set(-SetShooterOutSpeed_P);
        //s_TalonRight.set(SetShooterOutSpeed_P);
      
      }else{
        //何も押してないならストップ
        i_Intakefront.set(0);
        i_Intakeback.set(0);
        i_Intakeroller.set(0);

        s_TalonLeft.set(0);
        s_TalonRight.set(0);
      }
      

    }else{

      //トリガーで発射
      i_Intakefront.set(SetIntakeBeltSpeed_P);
      s_TalonLeft.set(ControlMode.Velocity, -SetSpeedPoint);
      s_TalonRight.set(ControlMode.Velocity, SetSpeedPoint);
    }

    
//--------------------------------------------------------------------------------------

    //砲台
    if(-m_xbox.getY(Hand.kRight) > 0.2){
      m_Talon.set(0.1);
    }

  }




  void S_TalonLeftPIDset(WPI_TalonSRX s_TalonLeft){
    //s_TalonLeft.set(ControlMode.Velocity, SetSpeedPoint);
  }

  void S_TalonRightPIDset(WPI_TalonSRX s_TalonRight){
    //s_TalonRight.set(ControlMode.Velocity, -SetSpeedPoint);
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
