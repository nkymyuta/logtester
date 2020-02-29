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
import com.ctre.phoenix.motorcontrol.DemandType;

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

   
  //アームの可動域の角度＆エンコーダーからの値の最大
    public static final double CanonMaxAngle   = 80;
    public static final double CanonMinAngle   = -30;
    public static final double CanonMaxPoint   = 500;   
    public static final double CanonMinPoint   = 167;

    public static final double CanonPointError = CanonMaxPoint - CanonMinPoint;
    public static final double CanonAngleError = CanonMaxAngle - CanonMinAngle;

  //アームの重力オフセット最大値（角度が地面と平行であり、Cos = 1の時）
    public static final double CanonMaxOffset = 0.13;


  //PIDの目標角度を代入する変数
  public double SetAngle = -30;

  //PIDするかどうか
  public boolean canonPID_ON = false;

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

    //PID設定
    //pidgain.s_TalonLeftPIDSet(s_TalonLeft);
    //pidgain.s_TalonRightPIDSet(s_TalonRight);

  //-------------------------------------------------------------------------------------
  //シューターPID左

        s_TalonLeft.configFactoryDefault();
        //s_TalonLeft.configOpenloopRamp(0.1);
        s_TalonLeft.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 
                                                 PidGain.kPIDLoopIdx, PidGain.kTimeoutMs);

        s_TalonLeft.setSensorPhase(true);
        //s_TalonLeft.setInverted(false);

        s_TalonLeft.configNominalOutputForward(0,PidGain.kTimeoutMs);
        s_TalonLeft.configNominalOutputReverse(0,PidGain.kTimeoutMs);
        s_TalonLeft.configPeakOutputForward(1.0,PidGain.kTimeoutMs);
        s_TalonLeft.configPeakOutputReverse(-1.0,PidGain.kTimeoutMs);

        s_TalonLeft.config_kF(PidGain.kPIDLoopIdx, PidGain.shootkF, PidGain.kTimeoutMs);
        s_TalonLeft.config_kP(PidGain.kPIDLoopIdx, PidGain.shootkP, PidGain.kTimeoutMs);
        s_TalonLeft.config_kI(PidGain.kPIDLoopIdx, PidGain.shootkI, PidGain.kTimeoutMs);
        s_TalonLeft.config_kD(PidGain.kPIDLoopIdx, PidGain.shootkD, PidGain.kTimeoutMs);

        s_TalonLeft.configMaxIntegralAccumulator(PidGain.kPIDLoopIdx,PidGain.shootMaxIntegralAccumulator);

  //----------------------------------------------------------
  //シューターPID右     
        s_TalonRight.configFactoryDefault();
        //s_TalonRight.configOpenloopRamp(0.1);
        s_TalonRight.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 
                                                  PidGain.kPIDLoopIdx, PidGain.kTimeoutMs);

        s_TalonRight.setSensorPhase(true);
        //s_TalonRight.setInverted(false);

        s_TalonRight.configNominalOutputForward(0,PidGain.kTimeoutMs);
        s_TalonRight.configNominalOutputReverse(0,PidGain.kTimeoutMs);
        s_TalonRight.configPeakOutputForward(1.0,PidGain.kTimeoutMs);
        s_TalonRight.configPeakOutputReverse(-1.0,PidGain.kTimeoutMs);

        s_TalonRight.config_kF(PidGain.kPIDLoopIdx, PidGain.shootkF, PidGain.kTimeoutMs);
        s_TalonRight.config_kP(PidGain.kPIDLoopIdx, PidGain.shootkP, PidGain.kTimeoutMs);
        s_TalonRight.config_kI(PidGain.kPIDLoopIdx, PidGain.shootkI, PidGain.kTimeoutMs);
        s_TalonRight.config_kD(PidGain.kPIDLoopIdx, PidGain.shootkD, PidGain.kTimeoutMs);

        s_TalonRight.configMaxIntegralAccumulator(PidGain.kPIDLoopIdx,PidGain.shootMaxIntegralAccumulator);

  //-------------------------------------------------------------------------------------
  //砲台
    m_Talon.configFactoryDefault();
    //s_TalonLeft.configOpenloopRamp(0.1);
    m_Talon.configSelectedFeedbackSensor(FeedbackDevice.Analog, 
                                         PidGain.kPIDLoopIdx, PidGain.kTimeoutMs);

    m_Talon.setSensorPhase(true);
    m_Talon.setInverted(true);

    m_Talon.configNominalOutputForward(0,PidGain.kTimeoutMs);
    m_Talon.configNominalOutputReverse(0,PidGain.kTimeoutMs);
    m_Talon.configPeakOutputForward(PidGain.CanonkPeakOutput, PidGain.kTimeoutMs);
    m_Talon.configPeakOutputReverse(-PidGain.CanonkPeakOutput, PidGain.kTimeoutMs);

    m_Talon.config_kF(PidGain.kPIDLoopIdx, PidGain.CanonkF, PidGain.kTimeoutMs);
    m_Talon.config_kP(PidGain.kPIDLoopIdx, PidGain.CanonkP, PidGain.kTimeoutMs);
    m_Talon.config_kI(PidGain.kPIDLoopIdx, PidGain.CanonkI, PidGain.kTimeoutMs);
    m_Talon.config_kD(PidGain.kPIDLoopIdx, PidGain.CanonkD, PidGain.kTimeoutMs);

    m_Talon.configMaxIntegralAccumulator(PidGain.kPIDLoopIdx,PidGain.CanonMaxIntegralAccumulator);

  //-------------------------------------------------------------------------------------
  //変数宣言
    SetAngle = 0;

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
    SmartDashboard.putBoolean("MaxDownSwitchClosed", MaxDownSwitch.isRevLimitSwitchClosed());
    SmartDashboard.putBoolean("MaxUpSwitchClosed",   MaxUpSwitch.isFwdLimitSwitchClosed());
    SmartDashboard.putBoolean("canonPID_ON", canonPID_ON);

    //SmartDashboard.putBoolean("BallSensorFront",BallSensorFront.get());
    //SmartDashboard.putBoolean("BallSensorBack", BallSensorBack.get());

    //SmartDashboard.putBoolean("GetBumper/L",m_xbox.getBumper(Hand.kLeft));
    //SmartDashboard.putBoolean("GetBumper/R",m_xbox.getBumper(Hand.kRight));


    SmartDashboard.putNumber("Xboxstick/X/L",m_xbox.getX(Hand.kLeft));
    SmartDashboard.putNumber("Xboxstick/Y/L",m_xbox.getY(Hand.kLeft));
    SmartDashboard.putNumber("Xboxstick/X/R",m_xbox.getX(Hand.kRight));
    SmartDashboard.putNumber("Xboxstick/Y/R",m_xbox.getY(Hand.kRight));


    SmartDashboard.putNumber("CanonEncoder_P",m_TalonEncoder.getAnalogInRaw());
    SmartDashboard.putNumber("CanonEncoder_V",m_TalonEncoder.getAnalogInVel());
    
    //SmartDashboard.putNumber("ShootEncoder/L_V",s_TalonEncoderLeft.getAnalogInVel());
    //SmartDashboard.putNumber("ShootEncoder/R_V",s_TalonEncoderRight.getAnalogInVel());
 
    //SmartDashboard.putNumber("SetshooterPoint",SetSpeedPoint);

    SmartDashboard.putNumber("CanonNowAngle",getCanonNow(m_TalonEncoder.getAnalogInRaw()));
    SmartDashboard.putNumber("SetAngle", SetAngle);
    SmartDashboard.putNumber("getIA",m_Talon.getIntegralAccumulator());

    //SetFeedForward(getCanonNow(m_TalonEncoder.getAnalogInRaw()));
    //SmartDashboard.putNumber("FeedForwardMagni", Math.cos(Math.toRadians(getCanonNow(m_TalonEncoder.getAnalogInRaw()))));

    //--------------------------------------------------------------------------------------    
    /*
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

       //前センサーが入力してる時(0)、後ろセンサーが入力してない時（1）だけバック回す
        if(!BallSensorFront.get() && BallSensorBack.get()){        
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
    */

//--------------------------------------------------------------------------------------

    /*
    //砲台
    if(-m_xbox.getY(Hand.kRight) > 0.2){
      m_Talon.set(0.1);
    }
    */
    
    
    //PIDのテスト
    if(m_xbox.getBButton()){      
      canonPID_ON = true;
      SetAngle = 0;    
    }
    else if(m_xbox.getAButton()){      
      canonPID_ON = false;
      ChangeBasic();
    }
    else if(m_xbox.getYButton()){      
      canonPID_ON = true;
      SetAngle = 30;
    }
    
    
    
    if(canonPID_ON == true){
    CanonPIDMove(SetAngle, getCanonNow(m_TalonEncoder.getAnalogInRaw()));
    }
    

    /*
    if(-m_xbox.getY(Hand.kRight) > 0.2){
      i_Intakeroller.set(m_xbox.getY(Hand.kRight) * 0.7);
    }else if(m_xbox.getY(Hand.kRight) > 0.2){
      i_Intakeroller.set(m_xbox.getY(Hand.kRight) * 0.7);
    }else{
      i_Intakeroller.set(0);
    }
    */


  }

  //砲台のモーターを回すPID制御(位置をSetPoint()で決める・重力オフセットをSetFeedForward()で決める)
  void CanonPIDMove(double TargetAngle, double NowAngle){      
      
      m_Talon.set(ControlMode.Position, SetPoint(TargetAngle), 
                  DemandType.ArbitraryFeedForward, SetFeedForward(NowAngle));

  }

  
  //一番下に向ける
  void ChangeBasic(){
   
    while(!MaxDownSwitch.isRevLimitSwitchClosed()){
      m_Talon.set(ControlMode.PercentOutput, -0.1);
    }

  }
  
  //---------------------------------------------------------------------
  //砲台角度制御に関する計算式
  
    //現在の砲台の角度を計算 
    //(角度の最大最小差分) ÷（エンコーダー値の最大最小差分) × (エンコーダー現在値 - 最小値) + (角度の最小値)
    double getCanonNow(int CanonNowPoint){
      double NowAngle;
      NowAngle = CanonAngleError / CanonPointError * (CanonNowPoint - CanonMinPoint) + CanonMinAngle;

      return NowAngle;
    }

    //目標角度に合わせたPIDの目標値を計算
    //(目標角度 - 最小角度) ×（エンコーダー値の最大最小差分) ÷ (角度の最大最小差分) + (0からの差分)
    double SetPoint(double TargetAngle){
      double Targetpoint;
      Targetpoint = (TargetAngle - CanonMinAngle) * CanonPointError / CanonAngleError + CanonMinPoint;

      SmartDashboard.putNumber("TargetPoint", Targetpoint);
      return  Targetpoint;
    }

    //目標角度に合わせた重力オフセットを計算
    //(地面と水平な時の重力オフセット) × (cos目標角度)
    double SetFeedForward(double NowAngle){
      double FeedForward;
      FeedForward = CanonMaxOffset * Math.cos(Math.toRadians(NowAngle));

      SmartDashboard.putNumber("TargetFeedForward", FeedForward);
      return FeedForward;
    }

  //--------------------------------------------------------------------------------------


  @Override
  public void testPeriodic() {
  }
}
