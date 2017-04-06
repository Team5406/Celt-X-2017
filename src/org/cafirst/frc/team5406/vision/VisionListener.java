package org.cafirst.frc.team5406.vision;

import org.cafirst.frc.team5406.robot.Constants;
import org.opencv.core.CvType;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;
import org.opencv.imgproc.Imgproc;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.vision.VisionRunner;

public class VisionListener implements VisionRunner.Listener<GripPipeline>{

	private double bigRectHeight = 0;
	private double smallRectHeight = 0;
	
	private Point topRightPoint = new Point();
	private Point bottomLeftPoint = new Point();
	
	private Point[] bigRectPoints = new Point[0];
	private Point[] smallRectPoints = new Point[0];
	
	private double length = 0;
	
	private boolean boilerVisible = false;
	
	private long frameCount = 0;
	
	@Override
	public void copyPipelineOutputs(GripPipeline pipeline) {
		
		SmartDashboard.putNumber("Random Vision", System.nanoTime());
        frameCount = pipeline.getFrameCount();
		
		if(!pipeline.findContoursOutput().isEmpty())
		{
	            
	 
	            MatOfPoint2f smallContour = new MatOfPoint2f();
	            MatOfPoint2f bigContour = new MatOfPoint2f();
	            
	            int small = 0;
	            
	            for(int i = 0; i < pipeline.findContoursOutput().size(); i++)
	            	if(Imgproc.boundingRect(pipeline.findContoursOutput().get(i)).width > 32 && Imgproc.boundingRect(pipeline.findContoursOutput().get(i)).height > 7)
	            	{
	            		small = i;
	            		break;
	            	}
	            
	            pipeline.findContoursOutput().get(small).convertTo(smallContour, CvType.CV_32F);
	            
	            Rect r = Imgproc.boundingRect(pipeline.findContoursOutput().get(small));          
	            int big = 0;
	            
	            for(int i = small + 1; i < pipeline.findContoursOutput().size(); i++)
	            	if(Imgproc.boundingRect(pipeline.findContoursOutput().get(i)).width > 32 && Imgproc.boundingRect(pipeline.findContoursOutput().get(i)).height > 7)
	            	{
	            		big = i;
	            		break;
	            	}
	            pipeline.findContoursOutput().get(big).convertTo(bigContour, CvType.CV_32F);
	            
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
	            
	            topRight = bigRectVertices[0].y < bigRectVertices[1].y ? bigRectVertices[0]
	            		: bigRectVertices[1];
	            
	            for(int i = 0; i < smallRectVertices.length - 1; i++)
	            	for(int y = i; y < smallRectVertices.length - 1; y++)
	            		if(smallRectVertices[y].x > smallRectVertices[y + 1].x)
	            		{
	            			Point temp = smallRectVertices[y];
	            			smallRectVertices[y] = smallRectVertices[y + 1];
	            			smallRectVertices[y + 1] = temp;
	            		}
	            
	            bottomLeft = smallRectVertices[0].y > smallRectVertices[1].y ? smallRectVertices[0]
	            		: smallRectVertices[1];
	            
	            topRightPoint = topRight;
	            bottomLeftPoint = bottomLeft;
	            
	            bigRectPoints = bigRectVertices;
	            smallRectPoints = smallRectVertices;
	            
	            bigRectHeight = bigHeight;
	            smallRectHeight = smallHeight;
	            
	            Constants.centerX = bottomLeft.x + ((topRight.x - bottomLeft.x) / 2);
	            length =  Math.sqrt(Math.pow(topRight.x - bottomLeft.x, 2) + Math.pow(topRight.y - bottomLeft.y, 2));
	            
	            SmartDashboard.putNumber("CenterX", Constants.centerX);
	            
	            SmartDashboard.putString("Top Right", topRight.x + " , " + topRight.y);
	            SmartDashboard.putString("Bottom Left", bottomLeft.x + " , " + bottomLeft.y);
	            
	            SmartDashboard.putNumber("SmallHeight", smallHeight);
	            SmartDashboard.putNumber("SmallWidth", smallHeight == smallRect.size.width ? smallRect.size.height : smallRect.size.width);
	            
	            SmartDashboard.putNumber("BigHeight", bigHeight);
	            SmartDashboard.putNumber("BigWidth", bigHeight == bigRect.size.width ? bigRect.size.height : bigRect.size.width);
	            
	            SmartDashboard.putNumber("Vision Length", length);
	            SmartDashboard.putNumber("Contours", pipeline.findContoursOutput().size());
	            boilerVisible = (Imgproc.boundingRect(pipeline.findContoursOutput().get(small)).width > 32) && (Imgproc.boundingRect(pipeline.findContoursOutput().get(big)).width > 32) && (Imgproc.boundingRect(pipeline.findContoursOutput().get(small)).height > 7) && (Imgproc.boundingRect(pipeline.findContoursOutput().get(big)).height > 7);
	            //frameCount++;
		}
		
		else
		{
			Constants.centerX = 0;
			boilerVisible = false;
		}
		SmartDashboard.putBoolean("boilerVisible", boilerVisible);
		
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
	
	public boolean getBoilerVisible()
	{
		return boilerVisible;
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
}
