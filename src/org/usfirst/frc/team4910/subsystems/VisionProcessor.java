package org.usfirst.frc.team4910.subsystems;

import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import org.opencv.videoio.VideoCapture;
import org.usfirst.frc.team4910.iterations.Iterate;
import org.usfirst.frc.team4910.robot.OI;
import org.usfirst.frc.team4910.robot.RobotMap;

import edu.wpi.cscore.AxisCamera;
import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.VideoProperty;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;
/**
 * 
 * @author Jason Cobb
 * 
 * Purpose is to capture an image when triggered and then calculate whatever distances necessary.
 * Because of the delay, it will use timestamps to determine how the data should be applied
 */
@SuppressWarnings("unused")
public class VisionProcessor {
	private static VisionProcessor instance;
	private static final double verticalFOV = 67.0;
	private static final double horizontalFOV = 49.0;
	private static final double resolutionY=320.0;
	private static final double resolutionX=240.0;
	private static final double degPerPixelX = horizontalFOV/resolutionX; //320x240
	private static final double degPerPixelY = verticalFOV/resolutionY;
	private static final double centerImgX = 119.5;
	private static final double centerImgY = 159.5;
	private static final double pegTapeWidth=2.0; //In inches.
	private static final double pegTapeHeight=5.0;
	private static final double pegTapeDistance=6.25; //Distance, in inches, from the tapes
	private static final double tapeArea = 10.0; //Just area of one
	private static final double aspectRatio = pegTapeWidth/pegTapeHeight;
	private static final double heightToPegCenter = 7.0; //In inches
	private static final double minPegContourArea=35;
	
	private static final double heightToGoalCenter=0.0; //distance in inches to the center of the tapes
	private static final double goalCamFromCenterX=0.0; //from a top view, the base of the triangle from camera position to robot center
	private static final double goalCamFromCenterY=0.0; //height of that same triangle
	/**
	 * 
	 * @param cD calculated distance in inches
	 * @param t Angle in degrees. Use angle calculated from yawAngleToTargetX()
	 * @return transitional distance
	 */
	private static double goalTransitionalDistance(double cD, double t){
		double X=cD*Math.sin(Math.toRadians(t));
		double Y=cD*Math.cos(Math.toRadians(t));
		double ans=Math.sqrt((X+goalCamFromCenterX)*(X+goalCamFromCenterX)+(Y+goalCamFromCenterY)*(Y+goalCamFromCenterY));
		return ans;
	}
	/**
	 * 
	 * @param cD calculated distance in inches
	 * @param t Angle in degrees. Use angle calculated from yawAngleToTargetX()
	 * @return transitional angle in degrees
	 */
	private static double goalTransitionalAngle(double cD, double t){
		double X=cD*Math.sin(Math.toRadians(t));
		double Y=cD*Math.cos(Math.toRadians(t));
		return Math.toDegrees(Math.atan((X+goalCamFromCenterX)/(Y+goalCamFromCenterY)));
	}
	
	private static HashMap<String, VideoProperty> propertyMap = new HashMap<String, VideoProperty>();
	private static double yawAngleToTargetApproxX(double error){return (error)*degPerPixelX;}
	private static double yawAngleToTargetApproxY(double error){return (error)*degPerPixelY;}
	private static final double focalLengthX = resolutionX/(2.0*Math.tan(Math.toRadians((horizontalFOV/2.0))));
	private static final double focalLengthY = resolutionY/(2.0*Math.tan(Math.toRadians((verticalFOV/2.0))));
	private static double yawAngleToTarget(double error){return Math.toDegrees(Math.atan((error)/focalLengthX));}
	private static double pitchAngleToTarget(double error){return Math.toDegrees(Math.atan((error)/focalLengthY));}
	private static double DistanceToTarget(double error){return heightToPegCenter*focalLengthY/error;}
	//tan(theta)=height/distance, tan(theta)=err/focalY, distance/height = focalY/err 
	private static double DistanceToTargetApprox(double error){return heightToPegCenter/Math.tan(Math.toRadians(error*degPerPixelY));}
	//distance=height/tan(theta)
	private static VideoCapture videoCapture;
	private static Mat BGR, HSV, blur, threshold, clusters, hierarchy;
	private static boolean insideTarget = false;
	private static SerialPort cam;
	private static CvSink cvs;
	private static double captureTime;
	private static AxisCamera axiscam; //shouldn't be used for much
	private static int iteration=0;
	private static boolean hasEnabled=false;
	private CameraServer camServer;
	private static double currentAngle=0.0;
	private static double currentDistance=0.0;
	private static double calculatedAngle=0.0;
	private static double calculatedDistance=0.0;
	private enum VisionStates {
			Disabled, PegTracking, BoilerTracking;
	}
	private VisionStates visionState=VisionStates.Disabled;
	private static final Scalar 
	//Color values
	BLUE = new Scalar(255, 0, 0),
	GREEN = new Scalar(0, 255, 0),
	RED = new Scalar(0, 0, 255),
	WHITE = new Scalar(255, 255, 255),
	BLACK = new Scalar(0, 0, 0),

	//HSV Threshold
	LOWER_BOUNDS = new Scalar(65,190,25),
	UPPER_BOUNDS = new Scalar(103,255,168);
	private static Scalar color = BLACK;
	private final Iterate iter = new Iterate(){
		
		@Override
		public void init() {
			synchronized(VisionProcessor.this){
				visionState=VisionStates.Disabled;
				hasEnabled=false;
			}
		}

		@Override
		public void exec() {
			synchronized(VisionProcessor.this){
				switch(visionState){
				case Disabled:
					if(OI.leftStick.getRawButton(OI.StartPegTrackingTest)){
						//no need to put a while loop, since next iteration it'll go to PegTracking
						startPegTracking();
					}
					break;
				case PegTracking:
					if(OI.leftStick.getRawButton(OI.StopPegTrackingTest)){
						stopPegTracking();
						break;
					}
					processPeg();
					break;
				case BoilerTracking:
					break;
				default:
					break;
				}
			}
		}

		@Override
		public void end() {
			synchronized(VisionProcessor.this){
				visionState=VisionStates.Disabled;
			}
		}
		
	};
	
	public Iterate getLoop(){
		return iter;
	}
	private VisionProcessor(){
		System.load("/usr/local/frc/lib/libopencv_java310.so");
		BGR = new Mat();
		HSV = new Mat();
		blur = new Mat();
		threshold = new Mat();
		clusters = new Mat();
		hierarchy = new Mat();
		
		
	}
	public static VisionProcessor getInstance(){
		return instance==null ? instance=new VisionProcessor() : instance;
	}
	public synchronized void startPegTracking(){
		synchronized(VisionProcessor.this){
			visionState=VisionStates.PegTracking;
			camServer = CameraServer.getInstance();
			axiscam = camServer.addAxisCamera("10.49.10.40"); //44 is high goal
			cvs = camServer.getVideo(axiscam);
			cvs.setEnabled(true);
			hasEnabled=true;
			cvs.grabFrame(BGR);
			captureTime=Timer.getFPGATimestamp(); //this way, distance can be adjusted for current robot pose
			currentAngle=RobotMap.spig.getAngle();
			boolean b = Imgcodecs.imwrite("/home/lvuser/pegStartingImage.png", BGR);
			System.out.println("Able to write image: "+ b);
			insideTarget = false;
		}
		
	
	}
	public synchronized void stopPegTracking(){
		synchronized(VisionProcessor.this){
			visionState=VisionStates.Disabled;
		}
	}
	private synchronized void processPeg(){
		synchronized(VisionProcessor.this){
			cvs.grabFrame(BGR);
			captureTime=Timer.getFPGATimestamp();
			currentAngle=RobotMap.spig.getAngle();
			currentDistance=(RobotMap.left1.getEncPosition()+RobotMap.right1.getEncPosition())/2.0;
			iteration++;
			ArrayList<MatOfPoint> contours = new ArrayList<MatOfPoint>();
			if(!BGR.empty() && !insideTarget){
				//System.out.println("Image not empty");
				Imgproc.cvtColor(BGR, HSV, Imgproc.COLOR_BGR2HSV); //turns image to HSV format
				Core.inRange(HSV, LOWER_BOUNDS, UPPER_BOUNDS, threshold); //applies HSV filter
				Imgproc.findContours(threshold, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE); //Finds contours

				double contA; //contour area


				if(hierarchy.size().height > 0 && hierarchy.size().width > 0 && !insideTarget){
					//System.out.println("Contours found");

					Rect rec1=null, rec2=null; //one for each peg
					Rect totalRect=null;
					boolean hasFoundAPeg=false;
					for(int idx = 0; idx >= 0 && !insideTarget; idx = (int) hierarchy.get(0, idx)[0]){
						contA = Imgproc.contourArea(contours.get(idx));
						if(contA > minPegContourArea && !insideTarget){
							//System.out.println("Large contours found");
							MatOfPoint approxf1 = new MatOfPoint();
							MatOfPoint2f mMOP2f1 = new MatOfPoint2f(); // Converted contours
							MatOfPoint2f mMOP2f2 = new MatOfPoint2f(); // approxPolyDP stored

							contours.get(idx).convertTo(mMOP2f1, CvType.CV_32FC2);
							Imgproc.approxPolyDP(mMOP2f1, mMOP2f2, 7, true);
							mMOP2f2.convertTo(contours.get(idx), CvType.CV_32S);

							mMOP2f2.convertTo(approxf1, CvType.CV_32S);
							Imgproc.drawContours(BGR, contours, idx, WHITE);
							if(!hasFoundAPeg){
								rec1 = Imgproc.boundingRect(approxf1);
								Point recMiddle = new Point(rec1.x+.5*rec1.width,rec1.y+.5*rec1.height);
								Imgproc.circle(BGR, recMiddle, 2, BLUE);
								hasFoundAPeg=true;
							}else{
								rec2 = Imgproc.boundingRect(approxf1);
								Point recMiddle = new Point(rec2.x+.5*rec2.width,rec2.y+.5*rec2.height);
								Imgproc.circle(BGR, recMiddle, 2, BLUE);
								if(Math.min(rec1.tl().x, rec2.tl().x)==rec1.tl().x){
									totalRect = new Rect(rec1.tl(), rec2.br());
								}else{
									totalRect = new Rect(rec2.tl(), rec1.br());
								}
								Imgproc.rectangle(BGR, totalRect.br(), totalRect.tl(), GREEN);
								recMiddle = new Point(totalRect.x+.5*totalRect.width,totalRect.y+.5*totalRect.height);
								Imgproc.circle(BGR, recMiddle, 2, RED);
								break; //no need to go further
							}

						}//end "check size" conditional inside for loop


					}//ends "iterate over contours" loop
					Imgcodecs.imwrite("/home/lvuser/output"+iteration+".png", BGR);
					try{
						double deltaAngle = RobotMap.spig.getAngle()-currentAngle;
						double deltaDistance = ((RobotMap.left1.getEncPosition()+RobotMap.right1.getEncPosition())/2.0) - currentDistance;
						//System.out.println("Ground distance to target: "+DistanceToTarget(totalRect.y+.5*totalRect.height));
						//Change in angle = old - new, angleSetpoint = calculatedAngle - deltaAngle
						//It would be twice the current - the old + calculated because of relative position, however the heading is reset when the path starts
						//calculatedAngle=RobotMap.spig.getAngle()+yawAngleToTargetX(totalRect.x+.5*totalRect.width)-currentAngle;
						//calculatedDistance=((RobotMap.left1.getEncPosition()+RobotMap.right1.getEncPosition())/2.0)+DistanceToTarget((totalRect.y+.5*totalRect.height))-currentDistance;
						//calculatedDistance=-257.75+(85.292/Math.atan(0.0280534*calculatedDistance)); 
						//calculatedDistance = 10.6804+38.2452/(185179*yawAngleToTargetY((totalRect.y+.5*totalRect.height))-3.66216);
						//I messed up somewhere in my calculations, but this fits the curve.
						calculatedAngle = yawAngleToTarget((totalRect.x+.5*totalRect.width)-centerImgX) + deltaAngle;
						double calculatedAngleY = pitchAngleToTarget((totalRect.y+.5*totalRect.height)-centerImgY);
						calculatedDistance = DistanceToTarget((totalRect.y+.5*totalRect.height)-centerImgY)+deltaDistance;
						
						System.out.println("Center X coordinate: "+(totalRect.x+.5*totalRect.width));
						System.out.println("Center Y coordinate: "+(totalRect.y+.5*totalRect.height));
						System.out.println("Angle to target X: "+calculatedAngle);
						System.out.println("Angle to target Y: "+calculatedAngleY);
						System.out.println("Ground distance to target: "+calculatedDistance);
						
					}catch(Exception e){
						System.out.println("No target found");
						calculatedAngle=0;
						calculatedDistance=0;
					}
				}//ends "find contours" conditionals

			}//ends "is image even there" conditional
		}//end sync block
	}//end of processing
	/**
	 * 
	 * @return angle to peg
	 */
	public synchronized double getCalculatedPegAngle(){
		return calculatedAngle;
	}
	
}
