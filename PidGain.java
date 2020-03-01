
package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
//import com.ctre.phoenix.motorcontrol.SensorCollection;
//import com.ctre.phoenix.motorcontrol.FeedbackDevice;


public class PidGain{

    /**
	 * Which PID slot to pull gains from. Starting 2018, you can choose from
	 * 0,1,2 or 3. Only the first two (0,1) are visible in web-based
	 * configuration.
	 */
    public static final int kSlotIdx = 0;

	/**
	 * Talon SRX/ Victor SPX will supported multiple (cascaded) PID loops. For
	 * now we just want the primary one.
	 */
	public static final int kPIDLoopIdx = 0;

	/**
	 * Set to zero to skip waiting for confirmation, set to nonzero to wait and
	 * report to DS if action fails.
	 */
    public static final int kTimeoutMs = 30;

    //shootPIDGain
    public static final double shootkP  = 0.01;
	public static final double shootkI  = 0.01;
	public static final double shootkD  = 0;
	public static final double shootkF  = 1023.0/7200.0;
	public static final int shootkIzone = 300;
    public static final double shootkPeakOutput = 1.00;
    public static final double shootMaxIntegralAccumulator = 2000000;

    //CanonPIDGain
    public static final double CanonkP  = 8;
	public static final double CanonkI  = 0.01;
	public static final double CanonkD  = 10;
	public static final double CanonkF  = 0;
	public static final int CanonkIzone = (int)(0.5 * 1023 / CanonkP);
    public static final double CanonkPeakOutput = 1.0;
    public static final double CanonMaxIntegralAccumulator  = 0.15 * 1023 / CanonkI;

//----------------------------------------------------------------------

    void s_TalonLeftPIDSet(WPI_TalonSRX s_TalonLeft){
       
        /*
        s_TalonLeft.setSensorPhase(true);
        s_TalonLeft.setInverted(false);

        s_TalonLeft.configNominalOutputForward(0,0);
        s_TalonLeft.configNominalOutputReverse(0,0);
        s_TalonLeft.configPeakOutputForward(1,0);
        s_TalonLeft.configPeakOutputReverse(-1,0);

        s_TalonLeft.config_kF(0,0.0,0);
        s_TalonLeft.config_kP(0,0.15,0);
        s_TalonLeft.config_kI(0,0.0,0);
        s_TalonLeft.config_kD(0,1.0,0);
        */

    }

    void s_TalonRightPIDSet(WPI_TalonSRX s_TalonRight){

        /*
        s_TalonRight.setSensorPhase(true);
        s_TalonRight.setInverted(false);

        s_TalonRight.configNominalOutputForward(0,0);
        s_TalonRight.configNominalOutputReverse(0,0);
        s_TalonRight.configPeakOutputForward(1,0);
        s_TalonRight.configPeakOutputReverse(-1,0);

        s_TalonRight.config_kF(0,0.0,0);
        s_TalonRight.config_kP(0,0.15,0);
        s_TalonRight.config_kI(0,0.0,0);
        s_TalonRight.config_kD(0,1.0,0);
        */
        
    }



}