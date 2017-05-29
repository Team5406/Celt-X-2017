package org.cafirst.frc.team5406.vision;

import org.opencv.core.CvType;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.imgproc.Imgproc;

import edu.wpi.cscore.CvSource;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.vision.VisionRunner;

public class VisionListener implements VisionRunner.Listener<GripPipeline>{
	
	private CvSource gripStream;
	
	private double centerX = 0;
	
	private double bigRectHeight = 0;
	private double smallRectHeight = 0;
	
	private Point[] bigRectPoints = new Point[0];
	private Point[] smallRectPoints = new Point[0];
	
	private Point topRightPoint = new Point();
	private Point bottomLeftPoint = new Point();
	
	private double length = 0;
	
	private boolean isBoilerVisible = false;
	
	private long frameCount = 0;
	
	/**Adds the GripStream to the Image Stream*/
	public VisionListener()
	{
		super();
		
		CameraServer.getInstance().removeServer("Grip");
		gripStream = CameraServer.getInstance().putVideo("Grip", 480, 360);
	}
	
	@Override
	public void copyPipelineOutputs(GripPipeline pipeline) 
	{
		SmartDashboard.putNumber("Random Vision", System.nanoTime());
		
		gripStream.putFrame(pipeline.hsvThresholdOutput());
		
		frameCount = pipeline.getFrameCount();
		
		if(!pipeline.findContoursOutput().isEmpty())
		{
			MatOfPoint2f smallContour = new MatOfPoint2f();
			MatOfPoint2f bigContour = new MatOfPoint2f();
			
			int smallIndex = 0;
			for(int i = 0; i < pipeline.findContoursOutput().size(); i++)
				if(Imgproc.boundingRect(pipeline.findContoursOutput().get(i)).width > 32 && Imgproc.boundingRect(pipeline.findContoursOutput().get(i)).height > 7)
				{
					smallIndex = i;
					break;
				}
			
			int bigIndex = 0;
			for(int i = smallIndex + 1; i < pipeline.findContoursOutput().size(); i++)
				if(Imgproc.boundingRect(pipeline.findContoursOutput().get(i)).width > 32 && Imgproc.boundingRect(pipeline.findContoursOutput().get(i)).height > 7)
				{
					bigIndex = i;
					break;
				}
			
			if(bigIndex < -1) bigIndex = smallIndex;
			
			pipeline.findContoursOutput().get(smallIndex).convertTo(smallContour, CvType.CV_32F);
			pipeline.findContoursOutput().get(bigIndex).convertTo(bigContour, CvType.CV_32F);
            
			RotatedRect smallRect = Imgproc.minAreaRect(smallContour);
            RotatedRect bigRect = Imgproc.minAreaRect(bigContour);
            
            double bigHeight = bigRect.size.width < bigRect.size.height ? bigRect.size.width : bigRect.size.height;
            double smallHeight = smallRect.size.width < smallRect.size.height ? smallRect.size.width : smallRect.size.height;
            
            if(bigHeight < smallHeight)
            {
            	RotatedRect temp = bigRect;
            	bigRect = smallRect;
            	smallRect = temp;
            }
            
            Point smallRectVertices[] = new Point[4];
            Point bigRectVertices[] = new Point[4];
            
            Point topRight = bigRectVertices[0];
            Point bottomLeft = smallRectVertices[0];
            
            smallRect.points(smallRectVertices);
            bigRect.points(bigRectVertices);
            
            for(int i = 0; i < bigRectVertices.length - 1; i++)
            	for(int y = 0; y < bigRectVertices.length - 1 - i; y++)
            		if(bigRectVertices[y].x < bigRectVertices[y + 1].x)
            		{
            			Point temp = bigRectVertices[y];
            			bigRectVertices[y] = bigRectVertices[y + 1];
            			bigRectVertices[y + 1] = temp;
            		}
            
            for(int i = 0; i < smallRectVertices.length - 1; i++)
            	for(int y = i; y < smallRectVertices.length - 1; y++)
            		if(smallRectVertices[y].x > smallRectVertices[y + 1].x)
            		{
            			Point temp = smallRectVertices[y];
            			smallRectVertices[y] = smallRectVertices[y + 1];
            			smallRectVertices[y + 1] = temp;
            		}
            
            topRight = bigRectVertices[0].y < bigRectVertices[1].y ? bigRectVertices[0]
            		: bigRectVertices[1];
            bottomLeft = smallRectVertices[0].y > smallRectVertices[1].y ? smallRectVertices[0]
            		: smallRectVertices[1];
            
            topRightPoint = topRight;
            bottomLeftPoint = bottomLeft;
            
            bigRectPoints = bigRectVertices;
            smallRectPoints = smallRectVertices;
            
            bigRectHeight = bigHeight;
            smallRectHeight = smallHeight;
            
            length =  Math.sqrt(Math.pow(topRight.x - bottomLeft.x, 2) + Math.pow(topRight.y - bottomLeft.y, 2));
            
            centerX = bottomLeft.x + ((topRight.x - bottomLeft.x) / 2);
            
            isBoilerVisible = (Imgproc.boundingRect(pipeline.findContoursOutput().get(smallIndex)).width > 32) && (Imgproc.boundingRect(pipeline.findContoursOutput().get(bigIndex)).width > 32) && (Imgproc.boundingRect(pipeline.findContoursOutput().get(smallIndex)).height > 7) && (Imgproc.boundingRect(pipeline.findContoursOutput().get(bigIndex)).height > 7);
		}else{
			centerX = 0;
			isBoilerVisible = false;
		}
		
		SmartDashboard.putBoolean("boilerVisible", isBoilerVisible);
	}
	
	public Point getTopRightPoint() {
		return topRightPoint;
	}

	public Point getBottomLeftPoint() {
		return bottomLeftPoint;
	}
	
	public double getLength()
	{
		return length;
	}
	
	public boolean isBoilerVisible()
	{
		return isBoilerVisible;
	}
	
	public Point[] getBigRectPoints()
	{
		return bigRectPoints;
	}
	
	public Point[] getSmallRectPoints()
	{
		return smallRectPoints;
	}
	
	public double getBigHeight()
	{
		return bigRectHeight;
	}
	
	public double getSmallHeight()
	{
		return smallRectHeight;
	}
	
	public long getFrameCount()
	{
		return frameCount;
	}
	
	public double getCenterX()
	{
		return centerX;
	}
}
