/*
 * Copyright (c) 2020 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.modules.opencv;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfInt;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;


// ToDo: Get localisation information for coneDetection to work in desired location
//  and avoid detecting cone on the poll.
/*
 * This is an advanced sample showcasing detecting and determining the orientation
 * of multiple Cones, switching the viewport output, and communicating the results
 * of the vision processing to user-code.
 */
public class ConeDetection
{
    ConeDetectionPipeline ConeDetectionpipeline;

    public final int RED_CONE  = 1;
    public final int BLUE_CONE = 2;

    public ConeDetection(OpenCvCamera camera) {

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()  {
            @Override
            public void onOpened()  {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
                ConeDetectionpipeline = new ConeDetectionPipeline(BLUE_CONE);
                camera.setPipeline(ConeDetectionpipeline);
            }

            @Override
            public void onError(int errorCode)  {    }
        });
    }

    public Rect getBoundary()
    {
        return ConeDetectionpipeline.getBoundary();
    }

    static class ConeDetectionPipeline extends OpenCvPipeline
    {
        int cone_type=-1;

        public Rect boundary = new Rect();

        public ConeDetectionPipeline(int cone_type)
        {
            cone_type=cone_type;
        }

        @Override
        public Mat processFrame(Mat frame)
        {
            // Make a working copy of the input matrix in HSV
            Mat imgHSV = new Mat();
            Mat coneMat = new Mat();

            Imgproc.cvtColor(frame, imgHSV, Imgproc.COLOR_RGB2HSV);
            //ToDo: Create different april Tag for RED and BLUE cones
            if(cone_type==1) {
                coneMat = redMask(imgHSV);
            }
            else if (cone_type==2) {
                coneMat = blueMask(imgHSV);
            }
            else
            {
                // ToDO return with error
            }

            Mat imgRGB = new Mat(); // do we need do this ???
            Mat imgGray = new Mat();

            Imgproc.cvtColor(coneMat, imgRGB, Imgproc.COLOR_HSV2RGB);
            Imgproc.cvtColor(imgRGB, imgGray, Imgproc.COLOR_RGBA2GRAY);

            // Use Canny Edge Detection to find edges
            Mat edges = new Mat();
            Imgproc.Canny(imgGray, edges, 100, 250);

            // https://docs.opencv.org/3.4/da/d0c/tutorial_bounding_rects_circles.html
            // Oftentimes the edges are disconnected. findContours connects these edges.
            // We then find the bounding rectangles of those contours
            List<MatOfPoint> contours = new ArrayList<MatOfPoint>();

            Mat hierarchy = new Mat();

            Imgproc.findContours(edges, contours, hierarchy, Imgproc.RETR_EXTERNAL,
                                 Imgproc.CHAIN_APPROX_SIMPLE);

            MatOfPoint2f[] contoursPoly  = new MatOfPoint2f[contours.size()];
            Rect[] boundRect = new Rect[contours.size()];

            for (int i = 0; i < contours.size(); i++) {
                contoursPoly[i] = new MatOfPoint2f();
                Imgproc.approxPolyDP(new MatOfPoint2f(contours.get(i).toArray()), contoursPoly[i], 3, true);
                boundRect[i] = Imgproc.boundingRect(new MatOfPoint(contoursPoly[i].toArray()));
            }


            for (int i = 0; i != boundRect.length; i++) {
                Imgproc.rectangle(frame, boundRect[i], new Scalar(0.5, 76.9, 89.8));
                boundary = boundRect[i];
            }

            return frame; // return the mat with rectangles drawn
        }

        public Rect getBoundary()
        {
            return boundary;
        }

        public Mat  blueMask(Mat imgHSV) {
            Scalar lowHSV = new Scalar(110, 50, 50); // lower bound HSV for blue
            Scalar highHSV = new Scalar(130, 255, 255); // higher bound HSV for blue
            Mat mask = new Mat();
            Core.inRange(imgHSV, lowHSV, highHSV, mask);
            return mask;
        }

        public Mat redMask(Mat imgHSV) {
            Scalar low1 = new Scalar(0, 70, 0);
            Scalar high1 = new Scalar(10, 255, 255);

            Scalar low2 = new Scalar(170, 70, 50);
            Scalar high2 = new Scalar(180, 255, 255);

            Mat mask1 = new Mat();
            Mat mask2 = new Mat();

            Core.inRange(imgHSV, low1, high1, mask1);
            Core.inRange(imgHSV, low2, high2, mask2);

            Mat mask = new Mat();
            //ToDO: Do we need this function for just ORing two maskes
            Core.addWeighted(mask1, 1.0, mask2, 1.0, 0.0, mask);

            return mask;
        }
    }
}