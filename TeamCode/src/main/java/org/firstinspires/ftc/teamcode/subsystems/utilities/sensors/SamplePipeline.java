package org.firstinspires.ftc.teamcode.subsystems.utilities.sensors;

import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;


public class SamplePipeline extends OpenCvPipeline{

    private static final double[] UPPER_COLOR =  {150,355,60}, LOWER_COLOR = {0,85,0};
    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, input,Imgproc.COLOR_RGB2BGR,0);
        Mat mask = createMask(input, UPPER_COLOR, LOWER_COLOR);
        Imgproc.cvtColor(mask, mask,Imgproc.COLOR_BGR2GRAY,0);
        Rect r = Imgproc.boundingRect(mask);
        Imgproc.rectangle (input, r, new Scalar(255,255,255));
        return input;

    }
    private Mat createMask(Mat input,double[] upper, double[] lower) {
        Mat output = new Mat();
        input.copyTo(output);

        for(int i = 0; i<input.rows();i++){
            for(int j = 0; j<input.cols();j++){
                double[] data = input.get(i,j);
                if ((data[2] < upper[2] && data[2] > lower[2] && data[1] < upper[1] && data[1] > lower[1] && data[0] < upper[0] && data[0] > lower[0])){
                    double[] black = {255,255,255};
                    output.put(i,j,black);

                }
                else{
                    double[] white = {0,0,0};
                    output.put(i,j,white);

                }
            }

        }
        return output;

    }


}