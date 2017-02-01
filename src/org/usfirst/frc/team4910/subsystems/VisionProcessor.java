package org.usfirst.frc.team4910.subsystems;

import java.io.IOException;

import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.videoio.VideoCapture;
import org.usfirst.frc.team4910.iterations.Iterate;

import edu.wpi.cscore.CvSink;
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
public class VisionProcessor {
	private VisionProcessor instance;
	private static final double horizontalFOV = 67.0;
	private static final double verticalFOV = 49.0;
	private static final double resolutionX=320.0; //I can change these in the future
	private static final double resolutionY=240.0;
	private static final double degPerPixelX = horizontalFOV/resolutionX; //320x240
	private static final double degPerPixelY = verticalFOV/resolutionY;
	private static final double centerImg = 159.5; //where the middle of the image is
	private static final double pegTapeWidth=2.0; //In inches.
	private static final double pegTapeHeight=5.0;
	private static final double pegTapeDistance=6.25; //Distance, in inches, from the tapes
	private static final double tapeArea = 10.0; //Just area of one
	private static final double aspectRatio = pegTapeWidth/pegTapeHeight;
	private static final double heightToPegCenter = 13.25; //In inches
	private static double yawAngleToTargetApproxX(double error){return (error)*degPerPixelX;}
	private static double yawAngleToTargetApproxY(double error){return (error)*degPerPixelY;}
	private static final double focalLengthX = resolutionX/(2.0*Math.tan(Math.toRadians((horizontalFOV/2.0))));
	private static final double focalLengthY = resolutionY/(2.0*Math.tan(Math.toRadians((verticalFOV/2.0))));
	private static double yawAngleToTargetX(double error){return Math.toDegrees(Math.atan((error)/focalLengthX));}
	private static double yawAngleToTargetY(double error){return Math.toDegrees(Math.atan((error)/focalLengthY));}
	private static double DistanceToTarget(double error){return heightToPegCenter*focalLengthY/error;}
	private static VideoCapture videoCapture;
	private static Mat BGR, HSV, blur, threshold, clusters, hierarchy;
	private static boolean insideTarget = false;
	private static SerialPort cam;
	private static CvSink cvs;
	private static int iteration=0;
	private CameraServer camServer;
	private static final Scalar 
	//Color values
	BLUE = new Scalar(255, 0, 0),
	GREEN = new Scalar(0, 255, 0),
	RED = new Scalar(0, 0, 255),
	WHITE = new Scalar(255, 255, 255),
	BLACK = new Scalar(0, 0, 0),

	//HSV Threshold
	LOWER_BOUNDS = new Scalar(74,94,25),
	UPPER_BOUNDS = new Scalar(103,255,168);
	private static Scalar color = BLACK;
	private final Iterate iter = new Iterate(){
		
		@Override
		public void init() {
			System.load("/usr/local/share/OpenCV/java/libopencv_java310.so");
			BGR = new Mat();
			HSV = new Mat();
			blur = new Mat();
			threshold = new Mat();
			clusters = new Mat();
			hierarchy = new Mat();
			camServer = CameraServer.getInstance();
			camServer.addAxisCamera("10.49.10.45");
			cvs = camServer.getVideo("cam2");
//			NetworkTable table = NetworkTable.getTable("SmartDashboard");
			
			cvs.setEnabled(true);
			cvs.grabFrame(BGR);
			boolean b = Imgcodecs.imwrite("/home/lvuser/output.png", BGR);
			System.out.println(b);
			insideTarget = false;
		}

		@Override
		public void exec() {
		}

		@Override
		public void end() {
		}
		
	};
	
	public Iterate getLoop(){
		return iter;
	}
	private VisionProcessor(){
		
	}
	public VisionProcessor getInstance(){
		return instance==null ? instance=new VisionProcessor() : instance;
	}
}
