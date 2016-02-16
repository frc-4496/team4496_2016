
package org.usfirst.frc.team4496.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import org.usfirst.frc.team4496.robot.commands.*;
import edu.wpi.first.wpilibj.Timer;


/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
	public static OI oi;

		//Start of added code
		Solenoid grabberArm;
		Compressor mainCompressor;
		RobotDrive mainDrive, testDrive;
		Victor liftDrive, catDrive;
		Command autoMode;
		SendableChooser autoChooser;
		Timer timArm, timArmAlt;

	    /**
	     * This function is run when the robot is first started up and should be
	     * used for any initialization code.
     	*/
    public void robotInit() {
    	
    	//Instantiate the command used for the autonomous period
        mainDrive = new RobotDrive(0, 1, 2, 3);
        mainDrive.setInvertedMotor(RobotDrive.MotorType.kFrontRight, true);
        mainDrive.setInvertedMotor(RobotDrive.MotorType.kRearRight, true);
        liftDrive = new Victor(4);
        
        //Add the defense choices to the Smart Dash-board
        autoChooser = new SendableChooser();
        autoChooser.addDefault("Bar Auto", new AutoBar());
        autoChooser.addObject("Terrain Auto", new AutoTerrain());
        autoChooser.addObject("Gate Auto", new AutoGate());        
        autoChooser.addObject("Wall Auto", new AutoWall());
        autoChooser.addObject("Sally Auto", new AutoSally());
        autoChooser.addObject("Cheval Auto", new AutoCheval());
        autoChooser.addObject("Moat Auto", new AutoMoat());
        autoChooser.addObject("Bridge Auto", new AutoBridge());
        autoChooser.addObject("Ramparts Auto", new AutoRamparts());
        SmartDashboard.putData("Auto Chooser", autoChooser);
        
        //Create the timers used in the grabber arm
        timArm = new Timer();
        timArmAlt = new Timer();
        
        //Pnumatics declarations
        mainCompressor = new Compressor();
        mainCompressor.setClosedLoopControl(false);
        grabberArm = new Solenoid(0);
        
        
    }
	
		/**
		 * This function is called once each time the robot enters Disabled mode.
		 * You can use it to reset any subsystem information you want to clear when
		 * the robot is disabled.
		 */
    public void disabledInit(){

    }
	
	public void disabledPeriodic() {
		Scheduler.getInstance().run();
	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select between different autonomous modes
	 * using the dashboard. The sendable chooser code works with the Java SmartDashboard. If you prefer the LabVIEW
	 * Dashboard, remove all of the chooser code and uncomment the getString code to get the auto name from the text box
	 * below the Gyro
	 *
	 * You can add additional auto modes by adding additional commands to the chooser code above (like the commented example)
	 * or additional comparisons to the switch structure below with additional strings & commands.
	 */
    public void autonomousInit() {
    	Scheduler.getInstance().run();
    	autoMode = (Command)autoChooser.getSelected();
    	autoMode.start();
    }

    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic() {
        Scheduler.getInstance().run();
    }

    public void teleopInit() {
    	//Start the timers used for the grabber and start the fly wheels
    	timArmAlt.start();
    	catDrive.set(1);
    }

    /**
     * This function is called periodically during operator control
     */
    @SuppressWarnings("deprecation")
	public void teleopPeriodic() {
        Scheduler.getInstance().run();
        //Main drive setup
        
        //Getting and rounding the input values
        double lXVal = OI.controller.getRawAxis(0);
        double lYVal = OI.controller.getRawAxis(1);
        double lTVal = OI.controller.getRawAxis(2);
        double rTVal = OI.controller.getRawAxis(3);
        double rXVal = OI.controller.getRawAxis(4);
        double rYVal = OI.controller.getRawAxis(5);
        
        //Slowing the drive by the triggers
        double sumTriggerValue = (lTVal + rTVal + 1) * 20;
        
        //Round and process the input 
        double rotDrv = ((double)((int)(lXVal  * 10)) ) / sumTriggerValue;
        double fwdDrv;
        if (Math.abs(lYVal) >= Math.abs(rYVal)) {
        	fwdDrv = ((double)((int)(lYVal * 10)) ) / sumTriggerValue;
        } else {
        	fwdDrv = ((double)((int)(rYVal * 10)) ) / sumTriggerValue;
        }
        double sldDrv = ((double)((int)(rXVal  * 10)) ) / sumTriggerValue;
        
        //SmartDashboard output
        SmartDashboard.putDouble("Rotational Drive Value", rotDrv);
        SmartDashboard.putDouble("Forward Drive Value", fwdDrv);
        SmartDashboard.putDouble("Sliding Drive Value", sldDrv);
        SmartDashboard.putInt("POV Value", OI.controller.getPOV());
        SmartDashboard.putBoolean("Compressor Status", !mainCompressor.getPressureSwitchValue());
        
        //Main drive controls
        mainDrive.mecanumDrive_Cartesian(rotDrv, fwdDrv, sldDrv, 0);
        
        //Compressor controls
        if(!mainCompressor.getPressureSwitchValue()){
        	mainCompressor.start();
        } else {
        	mainCompressor.stop();
        }
        
        //Grabber arm controls
        if (OI.controller.getRawButton(5) && timArm.get() == 0 && timArmAlt.get() >= 1) {
        	timArm.start();
        	timArmAlt.stop();
        	timArmAlt.reset();
        	grabberArm.set(true);
        } else if(OI.controller.getRawButton(5) && timArm.get() >= 1 && timArmAlt.get() == 0) {
        	timArm.stop();
        	timArmAlt.start();
        	timArm.reset();
        	grabberArm.set(false);
        }
        
        //Lifter Code
        if(OI.controller.getPOV() == 180){
        	liftDrive.set(-1);
        } else if(OI.controller.getPOV() == 0) {
        	liftDrive.set(1);
        } else {
        	liftDrive.set(0);
        }
       
    }
    
    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {
    }
}
