package org.firstinspires.ftc.teamcode.vision;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Locale;
import java.util.function.DoubleSupplier;

public class ColorBrightness extends OpenCvPipeline {


	private HardwareMap hardwareMap;
	private Telemetry telemetry;
	private OpenCvCamera camera;

	public enum TSEPosition {
		LEFT,      //here are the team prop positions, written as an enum
		MIDDLE,     //NOTE: team prop = team scoring element = TSE  (it all means the same)

		RIGHT

	}

	public enum TSEColor {
		RED,
		BLUE,

		BLANK
	}

	private static boolean midBlank = false;
	private static boolean leftBlank = false;

	// TOPLEFT anchor point for the bounding box

	private static Point MIDDLE_ANCHOR_POINT = new Point(200, 210); //the top left points
	private static Point LEFT_ANCHOR_POINT = new Point(50, 210);    //for each vision box
	private static Point RIGHT_ANCHOR_POINT = new Point(500, 270);
	private static int COLOR_DIFF = 10000;

	// Width and height for the bounding box
	public static int REGION_WIDTH = 25;//10          //the size of each vision box (there are going to
	public static int REGION_HEIGHT = 40;         //be 3; one for each spike position)

	// Color definitions
	private final Scalar

			RED = new Scalar(255, 0, 0),
			BLUE = new Scalar(0, 0, 255),
			GREY = new Scalar(127, 127, 127);

	Point pos1_pointA = new Point(             //create a top left and bottom right anchor point
			MIDDLE_ANCHOR_POINT.x,             //for each of the three boxes
			MIDDLE_ANCHOR_POINT.y);
	Point pos1_pointB = new Point(
			MIDDLE_ANCHOR_POINT.x + REGION_WIDTH,
			MIDDLE_ANCHOR_POINT.y + REGION_HEIGHT);

	Point pos2_pointA = new Point(
			LEFT_ANCHOR_POINT.x,
			LEFT_ANCHOR_POINT.y);
	Point pos2_pointB = new Point(
			LEFT_ANCHOR_POINT.x + REGION_WIDTH,
			LEFT_ANCHOR_POINT.y + REGION_HEIGHT);

	Point pos3_pointA = new Point(
			RIGHT_ANCHOR_POINT.x,
			RIGHT_ANCHOR_POINT.y);
	Point pos3_pointB = new Point(
			RIGHT_ANCHOR_POINT.x + REGION_WIDTH,
			RIGHT_ANCHOR_POINT.y + REGION_HEIGHT);

	// Running variable storing the parking position
	private volatile TSEPosition position = TSEPosition.MIDDLE;  //preset the position to middle
	private volatile TSEColor color = TSEColor.BLUE;

	public ColorBrightness(HardwareMap hw, Telemetry tele) {
		this.hardwareMap = hw;
		this.telemetry = tele;
	}

	public void runPipeline() {
		int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
		camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
		//could potentially look at createInternalCamera

		camera.setPipeline(this);
		camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
			@Override
			public void onOpened() {
				//camera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
				camera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
			}

			@Override
			public void onError(int errorCode) {
				throw new RuntimeException("Error opening camera! Error code " + errorCode);
			}
		});
	}


	@Override
	public Mat processFrame(Mat input) {  //NOTE: R = red, G = green, and B = blue

		position = TSEPosition.RIGHT;     //automatically assume the TSE is on the right
		color = TSEColor.BLUE;

		leftBlank = false;
		midBlank = false;

		Mat middleMat = input.submat(new Rect(pos1_pointA, pos1_pointB));     //create a matrix of pixels for the middle vision box
		Scalar midSumColors = Core.sumElems(middleMat);                       //sum all the colors of R, G, and B separately

		double maxColor = Math.max(midSumColors.val[0], Math.max(midSumColors.val[1], midSumColors.val[2]));  //see which is most prominent: R, G, or B

		if (midSumColors.val[0] > 40000 && midSumColors.val[1] > 40000 && midSumColors.val[2] > 40000) {         //if there's great lighting, the sums will be greater
			COLOR_DIFF = 20000;                                                                               //so the difference in R, G, and B will be greater
		} else if (midSumColors.val[0] < 40000 && midSumColors.val[1] < 40000 && midSumColors.val[2] < 40000) { //if there's bad lighting, the sums are less
			COLOR_DIFF = 15000;                                                                               //so the differences are less
		}
		telemetry.addData("MIDDLE RED COLOR", String.valueOf(midSumColors.val[0])); //see what the sums are in Logcat
		telemetry.addData("MIDDLE GREEN COLOR", String.valueOf(midSumColors.val[1]));
		telemetry.addData("MIDDLE BLUE COLOR", String.valueOf(midSumColors.val[2]));

		// Change the bounding box color based on the sleeve color
		if (midSumColors.val[0] == maxColor &&                                           //if the color is mostly RED out of RED GREEN and BLUE
				Math.abs(midSumColors.val[0] - midSumColors.val[1]) > COLOR_DIFF &&        //and there is a substantial difference between RED and GREEN/BLUE
				Math.abs(midSumColors.val[0] - midSumColors.val[2]) > COLOR_DIFF) {
			position = TSEPosition.MIDDLE;                                               //TSE is red and on the middle spike
			color = TSEColor.RED;
			Imgproc.rectangle(
					input,
					pos1_pointA,
					pos1_pointB,
					RED,
					2
			);
		} else if (midSumColors.val[2] == maxColor &&                                    //if the color is mostly BLUE out of RED GREEN and BLUE
				Math.abs(midSumColors.val[2] - midSumColors.val[0]) > COLOR_DIFF &&        //and there is a substantial difference between BLUE and GREEN/BLUE
				Math.abs(midSumColors.val[2] - midSumColors.val[1]) > COLOR_DIFF) {
			position = TSEPosition.MIDDLE;                                               //TSE is blue and on the middle spike
			color = TSEColor.BLUE;
			Imgproc.rectangle(
					input,
					pos1_pointA,
					pos1_pointB,
					BLUE,
					2
			);

		} else {
			//if it couldn't find the TSE on this spike,
			Imgproc.rectangle(                                                           //it must be somewhere else
					input,
					pos1_pointA,
					pos1_pointB,
					GREY,
					2
			);
		}


		//repeat what was done for the middle vision box and do it for the left vision box

		Mat leftMat = input.submat(new Rect(pos2_pointA, pos2_pointB));
		Scalar leftSumColors = Core.sumElems(leftMat);

		maxColor = Math.max(leftSumColors.val[0], Math.max(leftSumColors.val[1], leftSumColors.val[2]));



		telemetry.addData("LEFT RED COLOR", String.valueOf(leftSumColors.val[0])); //see what the sums are in Logcat
		telemetry.addData("LEFT GREEN COLOR", String.valueOf(leftSumColors.val[1]));
		telemetry.addData("LEFT BLUE COLOR", String.valueOf(leftSumColors.val[2]));

		// Change the bounding box color based on the sleeve color
		if (leftSumColors.val[0] == maxColor && Math.abs(leftSumColors.val[0] - leftSumColors.val[1]) > COLOR_DIFF && Math.abs(leftSumColors.val[0] - leftSumColors.val[2]) > COLOR_DIFF) {
			position = TSEPosition.RIGHT;
			color = TSEColor.RED;
			Imgproc.rectangle(
					input,
					pos2_pointA,
					pos2_pointB,
					RED,
					2
			);
		} else if (leftSumColors.val[2] == maxColor && Math.abs(leftSumColors.val[2] - leftSumColors.val[0]) > COLOR_DIFF && Math.abs(leftSumColors.val[2] - leftSumColors.val[1]) > COLOR_DIFF) {
			position = TSEPosition.RIGHT;
			color = TSEColor.BLUE;
			Imgproc.rectangle(
					input,
					pos2_pointA,
					pos2_pointB,
					BLUE,
					2
			);
		} else {
			Imgproc.rectangle(
					input,
					pos2_pointA,
					pos2_pointB,
					GREY,
					2
			);

		}



		//if it didn't find the TSE on the middle or left spikes, it has to be on the right spike.
		//the code for the right vision box doesn't assign a new position for the TSE even if it sees it
		//since we automatically assume already that the TSE is on the right spike

		//this code instead draws a box depending on the color it sees (like the code for the middle and left vision boxes)

            Mat rightMat = input.submat(new Rect(pos3_pointA, pos3_pointB));
            Scalar rightSumColors = Core.sumElems(rightMat);

            maxColor = Math.max(rightSumColors.val[0], Math.max(rightSumColors.val[1], rightSumColors.val[2]));

            // Change the bounding box color based on the sleeve color
            if (rightSumColors.val[0] == maxColor && Math.abs(rightSumColors.val[0]-leftSumColors.val[1]) > COLOR_DIFF && Math.abs(leftSumColors.val[0]-leftSumColors.val[2])>COLOR_DIFF) {
                position = TSEPosition.RIGHT;
                color = TSEColor.RED;
                Imgproc.rectangle(
                        input,
                        pos3_pointA,
                        pos3_pointB,
                        RED,
                        2
                );
            } else if (rightSumColors.val[2] == maxColor && Math.abs(rightSumColors.val[2]-leftSumColors.val[0]) > COLOR_DIFF && Math.abs(leftSumColors.val[2]-leftSumColors.val[1])>COLOR_DIFF) {
                position = TSEPosition.RIGHT;
                color = TSEColor.BLUE;
                Imgproc.rectangle(
                        input,
                        pos3_pointA,
                        pos3_pointB,
                        BLUE,
                        2
                );
            } else {
                Imgproc.rectangle(
                        input,
                        pos3_pointA,
                        pos3_pointB,
                        GREY,
                        2
                );
            }

        if(color==TSEColor.RED) {
            if(Math.abs(midSumColors.val[0]-midSumColors.val[1]) > leftSumColors.val[0]-leftSumColors.val[1]
                    && Math.abs(midSumColors.val[0]-midSumColors.val[2]) > rightSumColors.val[0]-rightSumColors.val[2]) {
                position = TSEPosition.MIDDLE;
            }
            if(Math.abs(leftSumColors.val[0]-leftSumColors.val[1]) > midSumColors.val[0]-midSumColors.val[1]
                    && Math.abs(leftSumColors.val[0]-leftSumColors.val[2]) > rightSumColors.val[0]-rightSumColors.val[2]) {
                position = TSEPosition.LEFT;
            }
            if(Math.abs(rightSumColors.val[0]-rightSumColors.val[1]) > midSumColors.val[0]-midSumColors.val[1]
                    && Math.abs(rightSumColors.val[0]-rightSumColors.val[2]) > leftSumColors.val[0]-leftSumColors.val[2]) {
                position = TSEPosition.RIGHT;
            }

        }
        if(color==TSEColor.BLUE) {
            if(Math.abs(midSumColors.val[2]-midSumColors.val[0]) > leftSumColors.val[2]-leftSumColors.val[0]
                    && Math.abs(midSumColors.val[2]-midSumColors.val[1]) > rightSumColors.val[2]-rightSumColors.val[1]) {
                position = TSEPosition.MIDDLE;
            }
            if(Math.abs(leftSumColors.val[2]-leftSumColors.val[0]) > midSumColors.val[2]-midSumColors.val[0]
                    && Math.abs(leftSumColors.val[2]-leftSumColors.val[1]) > rightSumColors.val[2]-rightSumColors.val[1]) {
                position = TSEPosition.LEFT;
            }
            if(Math.abs(rightSumColors.val[2]-rightSumColors.val[0]) > midSumColors.val[2]-midSumColors.val[0]
                    && Math.abs(rightSumColors.val[2]-rightSumColors.val[1]) > leftSumColors.val[2]-leftSumColors.val[1]) {
                position = TSEPosition.RIGHT;
            }
        }



		middleMat.release();                    //ALWAYS RELEASE THE MATRIX
		leftMat.release();
		rightMat.release();


		return input;      //return the input matrix and feed it back out of the processing frame
	}
	public void cleanup() {
		camera.closeCameraDevice();
	}

	//returns where the TSE is, so that the bot can use it to determine where it will place the pixel.
	public int getPosition() {
		switch (position) {
			case MIDDLE:
				return 2;
			case LEFT:
				return 1;
			case RIGHT:
				return 3;
		}
		return 0;
	}
	public TSEColor getColor() {
		return this.color;
	}

}


