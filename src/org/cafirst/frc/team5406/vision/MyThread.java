package org.cafirst.frc.team5406.vision;

import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

import edu.wpi.cscore.AxisCamera;
import edu.wpi.cscore.CvSink;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.cafirst.frc.team5406.robot.Constants;

public class MyThread extends Thread {
	
	//Filters Image
	GripPipeline gripPipeline;
	//Axis Camera
	AxisCamera axisCamera;
	
	//Used to filter
	Mat source = new Mat();
    
	//Converts Image into Mat
    CvSink cvSink;
    
    /*final double BOILER_HEIGHT = 160;
    final double CAMERA_WIDTH = 320;
    final double BOILER_DIAMETER = 15;
    final double WIDTH_ANGLE = 0.5375614;
    final double CAMERA_ANGLE = 0.698132; //Angle Camera is at
    
    double contourWidth = 0;
    double imageWidth = 0;
    double contourDistance = 0;
    
    
    double distance = 0;*/
    
    
	/**
	 * Test Thread Used to mimic Vision Thread
	 * @param gripPipeline
	 * @param axisCamera
	 */
	public MyThread(GripPipeline gripPipeline, AxisCamera axisCamera)
	{
		this.gripPipeline = gripPipeline;
		this.axisCamera = axisCamera;
		cvSink = CameraServer.getInstance().getVideo(axisCamera);
	}
	
	@Override
	public void run()
	{
		System.out.println("Thread Running");
		
		while(!interrupted())
		{
			
			//Puts frame into source
			cvSink.grabFrame(source);
			
			//Filters source
			gripPipeline.process(source);
			
			//Sets centerX to value (Please apply a synchronized block somehwere here
			if(!gripPipeline.findContoursOutput().isEmpty()) {
				//Creates a rectangle for the Contour
	            Rect r = Imgproc.boundingRect(gripPipeline.findContoursOutput().get(0));
	            //Finds the value of the center of the Tape
	            Constants.centerX = (r.x + (r.width / 2));
	            
	            SmartDashboard.putNumber("Width", r.width);
	            SmartDashboard.putNumber("Tape Y", r.y);
	            
	            SmartDashboard.putBoolean("isTapeVisible", true);
	            
	            /*contourWidth = r.width;
	            
	            imageWidth = (BOILER_DIAMETER / contourWidth) * CAMERA_WIDTH;
	            contourDistance = (imageWidth / 2) / (Math.tan(WIDTH_ANGLE));
	            
	            distance = Math.cos(CAMERA_ANGLE) * contourDistance;
	            
	            SmartDashboard.putNumber("Image Width", imageWidth);
	            SmartDashboard.putNumber("ContourDistance", contourDistance);
	            SmartDashboard.putNumber("Distance", distance);*/
	        }
			else{
				Constants.centerX = 0;
				SmartDashboard.putBoolean("isTapeVisible", false);
			}
			//Puts Video Stream for contours
			
			
			
		}
	}

}
