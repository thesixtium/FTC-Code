package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import java.util.Arrays;
import com.qualcomm.robotcore.util.Range;
import java.lang.reflect.Array;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cAddr;
import java.util.List;
import android.graphics.Bitmap;
import java.util.ArrayList;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
@Autonomous(name="VisionCodeCopyAndPaste")
@Disabled
public class VisionCodeCopyAndPaste extends LinearOpMode{
    int colorSide = 1;
    private VuforiaLocalizer vuforia;
    boolean farTest, centerTest, closeTest;
    int testxStart = 351, testyStart = 187, testwidth = 722, testheight = 120;
    //Telemetry outputs the average red value, average green value, average blue value,
    //number of yellow pixels, and number of black pixels.
    //we found the number of yellow pixels to be more accurate so that is what this
    //file does.
    //in order to run you need to define the pixel locations of the blocks... the pixels
    //must be measured from the bottom right pixel, though phone orientation may affect this
    //the test width is for the entire 3 block row. We found the best way to find these
    //measurements by opening up the built in Concept: VuMark Id and using a ruler to measure
    //the distance from the bottom right corner. Then use the entire distance across the view
    //to get the total x distance and using the fact that the camera is 1080x720 to get
    //the partial x distance. This is repeated to get every number and then fine tuned.
    public void runOpMode() throws InterruptedException{
        initVuforia();

        waitForStart();
        while(opModeIsActive()){
            findSkystone();
            telemetry.update();
        }
    }
    private void initVuforia() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = "AaNw5yn/////AAABmTPlCEPTXU4qsGbvT4c4pJkkfe22zDBnziTeK5i9UQn1uyHAWKWoN66qJrmWDWLBMAC/v0bdnpECUWVfAQciebhC4RSEQdvBJMoLNAA8+mqUgb7Q53oKKR267q8r4PdvMj8nQ/L8bpXQs/ohKNy2j/u+Ce8++dWdX+XynrmxMz4G9nKgp5ZPSO2NK9yJ8vCBmqJy307nfzQ/pK9n5gj2UOpe19IikYuSte5meyM5H6OR0JwnVQwN3xgTy3baT/bxz9aaK1CGkp5N1IOKg8fsx8lEXw94IVdQSI9oPjFhZKZG1wZfHt1/P97CzOcJ6CWUtnhJg2azB0doemtTwo30MIH24GNrY5LfEKkNbLZUE4gg";
        parameters.cameraDirection = CameraDirection.BACK;
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
        vuforia.setFrameQueueCapacity(1);
        vuforia.enableConvertFrameToBitmap();

    }
    public void findSkystone() throws InterruptedException{

        farTest=false;
        centerTest=false;
        closeTest=false;

        Bitmap bitmap = vuforia.convertFrameToBitmap(vuforia.getFrameQueue().take());
        int width = bitmap.getWidth();
        int height = bitmap.getHeight();
        int[] rawColorArray = new int[bitmap.getWidth()*bitmap.getHeight()];
        bitmap.getPixels(rawColorArray,0,width,0,0,width,height);
        double[][][] pixels = new double[width][height][3];
        for(int j = 0; j<height; j++){
            for(int i = 0; i<width; i++){
                //telemetry.addLine("width: " + i + " height: " + j + " value: "+((rawColorArray[j*width+i])&0xFF));
                pixels[i][j][0] = ((rawColorArray[j*width+i]>>16)&0xFF);
                pixels[i][j][1] = ((rawColorArray[j*width+i]>>8)&0xFF);
                pixels[i][j][2] = ((rawColorArray[j*width+i])&0xFF);
            }
        }
        int increment = testwidth/3;
        double[] block1Test = averageValues(pixels, testxStart+10, increment-20, testyStart, testheight);
        double[] block2Test = averageValues(pixels, testxStart+increment+10, increment-20, testyStart, testheight);
        double[] block3Test = averageValues(pixels, testxStart+increment*2+10, increment-20, testyStart, testheight);
        //r,g,b,yellows,blacks
        telemetry.addLine(Arrays.toString(block1Test));
        telemetry.addLine(Arrays.toString(block2Test));
        telemetry.addLine(Arrays.toString(block3Test));

        if(block1Test[3]<block2Test[3]&&block1Test[3]<block3Test[3]){
            if(colorSide==1){
                closeTest=true;
            }else{
                farTest=true;
            }
        }else if(block2Test[3]<block1Test[3]&&block2Test[3]<block3Test[3]){
            if(colorSide==1){
                centerTest=true;
            }else{
                centerTest=true;
            }
        }else if(block3Test[3]<block2Test[3]&&block3Test[3]<block1Test[3]){
            if(colorSide==1){
                farTest=true;
            }else{
                closeTest=true;
            }
        }
        telemetry.addLine(("close "+closeTest)+("   center "+centerTest)+("    far "+farTest));
        telemetry.update();
    }
    public double[] averageValues(double[][][] pixels, int xStart, int xLength, int yStart, int yLength){
        double[] temp = new double[5];
        int yellows = 0;
        int blacks = 0;


        double numPixels = xLength * yLength;
        for(int j = yStart; j<=(yStart+yLength); j++){
            for(int i = xStart; i<=(xStart+xLength); i++){
                //telemetry.addLine("j: " + j + " i: " + i + "Values: " + pixels[i][j][0] +","+ pixels[i][j][1]+","+ pixels[i][j][2]);
                temp[0]+=pixels[i][j][0];
                temp[1]+=pixels[i][j][1];
                temp[2]+=pixels[i][j][2];
            }
        }
        temp[0]/=numPixels;
        temp[1]/=numPixels;
        temp[2]/=numPixels;
        temp[0] = Math.round(temp[0]);
        temp[1] = Math.round(temp[1]);
        temp[2] = Math.round(temp[2]);

        for(int j = yStart; j<=(yStart+yLength); j++){
            for(int i = xStart; i<=(xStart+xLength); i++){
                //telemetry.addLine("j: " + j + " i: " + i + "Values: " + pixels[i][j][0] +","+ pixels[i][j][1]+","+ pixels[i][j][2]);
                if(pixels[i][j][0]>90&&pixels[i][j][1]>90&&pixels[i][j][2]<120&&pixels[i][j][0]+pixels[i][j][1]>pixels[i][j][2]*2.7){
                    yellows++;
                }
                if(pixels[i][j][0]<100&&pixels[i][j][1]<100&&pixels[i][j][2]<100&&(pixels[i][j][0]+pixels[i][j][1]+pixels[i][j][2])/3<75){
                    blacks++;
                }
            }
        }
        temp[3] = yellows;
        temp[4] = blacks;
        return temp;
    }
}