/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team3630.robot;

import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.AnalogInput;

import org.opencv.imgproc.Imgproc;
import org.usfirst.frc.team3630.robot.MyVisionPipeline;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.vision.VisionThread;
import edu.wpi.first.wpilibj.vision.VisionRunner;
import org.opencv.core.Rect;
import org.opencv.core.Mat;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in the
 * project.
 */
public class Robot extends IterativeRobot {
	private static final String kDefaultAuto = "Default";
	private static final String kCustomAuto = "My Auto";
	private String m_autoSelected;
	private SendableChooser<String> m_chooser = new SendableChooser<>();
	private OI oi = new OI();
	private UsbCamera camera;
	
	private MyVisionPipeline pipeline = new MyVisionPipeline();
	CvSink imageSink;
	CvSource imageSource;
	private boolean stopThread;
	
	private double[] centerX, centerY, size, height, width;
	private int contours;
	final static Object imgLoc = new Object();
	private VisionThread vision;
	
	public static AnalogInput touchless = new AnalogInput(0);
	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		m_chooser.addDefault("Default Auto", kDefaultAuto);
		m_chooser.addObject("My Auto", kCustomAuto);
		SmartDashboard.putData("Auto choices", m_chooser);
		
		camera = CameraServer.getInstance().startAutomaticCapture();
		camera.setResolution(Constants.img_width, Constants.img_height);
		
		new Thread(() -> {
		camera.setFPS(30);
		camera.setExposureAuto();
		
		CvSink cvSink = CameraServer.getInstance().getVideo();
		CvSource outputStream = CameraServer.getInstance().putVideo("Camera1", 640, 480); 
		//set up a new camera with this name in SmartDashboard (Veiw->Add->CameraServer Stream Viewer)
		
		Mat source = new Mat();
		Mat output = new Mat();
		
		while(!Thread.interrupted())
		{
			cvSink.grabFrame(source);
			Imgproc.cvtColor(source, output, Imgproc.COLOR_BGR2RGB);
			outputStream.putFrame(output);
		}					
	}).start(); //camera init + execution
		
		vision = new VisionThread(camera, new MyVisionPipeline(), pipeline ->  {
			if(oi.startB.get())
			{
				System.out.println("running vision");
				contours = pipeline.filterContoursOutput().size();
				centerX = new double[contours];
				centerY = new double[contours];
				size = new double[contours];
				height = new double[contours];
				width = new double[contours];
				System.out.println(contours);
				if(!pipeline.filterContoursOutput().isEmpty())
				{
					for(int i = 0; i<contours; i++)
					{
						System.out.println("processing vision");
						Rect r = Imgproc.boundingRect(pipeline.filterContoursOutput().get(i));
						centerX[i] = r.x+(r.width/2);
						centerY[i] = r.y+(r.height/2);
						size[i] = r.area();
						height[i] = r.height;
						width[i] = r.width;
						System.out.println(centerX[i]);
					}
					SmartDashboard.putNumberArray("centerX", centerX);
					SmartDashboard.putNumberArray("area", size);
				}
			}
		});
		vision.start();

	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * getString line to get the auto name from the text box below the Gyro
	 *
	 * <p>You can add additional auto modes by adding additional comparisons to
	 * the switch structure below with additional strings. If using the
	 * SendableChooser make sure to add them to the chooser code above as well.
	 */
	@Override
	public void autonomousInit() {
		m_autoSelected = m_chooser.getSelected();
		// autoSelected = SmartDashboard.getString("Auto Selector",
		// defaultAuto);
		System.out.println("Auto selected: " + m_autoSelected);
	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {
	}
	
	@Override
	public void teleopInit()
	{
		stopThread = false;
	}
	
	/**
	 * This function is called periodically during operator control.
	 */
	@Override
	public void teleopPeriodic() {
		SmartDashboard.putNumber("encoder", touchless.getValue());
	}

	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {
	}
}
