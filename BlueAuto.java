package org.firstinspires.ftc.teamcode;

//Imports

import android.graphics.Bitmap;
import android.graphics.Color;
import android.view.View;
import java.util.Arrays;
import java.util.Locale;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaBase;



@Autonomous(name = "BlueAuto", group = "Framework")
@Disabled

public class BlueAuto extends LinearOpMode {

    //Hardware Initializing

    private Servo clawClampServo;
    private Servo capstoneServo;
    private Servo zAxisClawSpinSevo;
    private Servo leftFrontClamp;
    private Servo rightFrontClamp;

    private DcMotor frontLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backLeftMotor;
    private DcMotor backRightMotor;

    private DcMotor yAxisClawMoveMotor;
    private DcMotor pullySystemMotor;

    private ColorSensor frontLeftColorSensor;
    private ColorSensor frontRightColorSensor;
    private DistanceSensor frontLeftDistanceSensor;
    private DistanceSensor frontRightDistanceSensor;


    //Variable Initializing
    int colorSide = 1;
    private VuforiaLocalizer vuforia;
    boolean farTest, centerTest, closeTest;
    int testxStart = 351, testyStart = 187, testwidth = 722, testheight = 120;
    int blockResult = 0;


    @Override
    public void runOpMode() throws InterruptedException {
        //Code to run ONE TIME after the driver hits INIT

        //Variable Initialization
        initVuforia();


        //Hardware Mapping
        clawClampServo           =  hardwareMap.servo.get("clawClamp");
        zAxisClawSpinSevo        =  hardwareMap.servo.get("zAxisClawSpin");
        capstoneServo            =  hardwareMap.servo.get("capstoneServo");
        rightFrontClamp          =  hardwareMap.servo.get("rightFrontClamp");
        leftFrontClamp           =  hardwareMap.servo.get("leftFrontClamp");

        frontLeftMotor           =  hardwareMap.dcMotor.get("frontLeft");
        frontRightMotor          =  hardwareMap.dcMotor.get("frontRight");
        backLeftMotor            =  hardwareMap.dcMotor.get("backLeft");
        backRightMotor           =  hardwareMap.dcMotor.get("backRight");

        pullySystemMotor         =  hardwareMap.dcMotor.get("pullySystemMotor");
        yAxisClawMoveMotor       =  hardwareMap.dcMotor.get("yAxisClawMove");

        frontLeftDistanceSensor  =  hardwareMap.get(DistanceSensor.class, "frontLeftDistanceSensor");
        frontRightDistanceSensor =  hardwareMap.get(DistanceSensor.class, "frontRightDistanceSensor");
        frontLeftColorSensor     =  hardwareMap.colorSensor.get("frontLeftColorSensor");
        frontRightColorSensor    =  hardwareMap.colorSensor.get("frontRightColorSensor");


        //Hardware Initialization
        frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        pullySystemMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        yAxisClawMoveMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        pullySystemMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        yAxisClawMoveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        //Set variables



        //Wait for start
        for(int i = 0; i < 50; ++i) {
            findSkystone();
            telemetry.update();
            telemetry.addData("Test Number: ", i);
            telemetry.addData("Block: ", blockResult);
            telemetry.update(); }
        sleep(5000);
        telemetry.addData("Block Results: ", blockResult);
        telemetry.addData("Status: ", "Done");
        telemetry.update();
        waitForStart();


        if (opModeIsActive()) {
            // Code to run ONCE when the driver hits PLAY

            /*
            //Use one of the get block functions
                //Move sideways if needed
            //Go forward until at block
            //Open Servo, grab block
            //Close Servo, go backwards
            // Drop off skystone
            // Get other skystone
            //Move to platform using on of the block functions
                //Drop off block WITHOUT stopping
            // Close servo
            //Move 90 degree and back to put in building stone, parallel to bridges
            // Back up to line
             */

            telemetry.update();

            telemetry.addLine("Starting initial block pathing");
            telemetry.update();
            if (blockResult == 0) { blockResult = 1;        }
            if (blockResult == 1) { pathFarthestFromWall(); }
            if (blockResult == 2) { pathMiddleToWall();     }
            if (blockResult == 3) { pathClosetToWall();     }

            telemetry.addLine("Moving forward to block");
            telemetry.update();
            forwardToBlock();

            telemetry.addLine("Opening Claw");
            telemetry.update();
            openClaw();

            telemetry.addLine("Moving forward a bit more");
            telemetry.update();
            for(int s=0; s < 500; ++s){driveFunction(0.5, 0);}
            driveFunction(0,0);

            telemetry.addLine("Closing Claw");
            telemetry.update();
            closeClaw();

            telemetry.addLine("Drive backwards");
            telemetry.update();
            for(int s=0; s < 1000; ++s){driveFunction(1, 180);}
            driveFunction(0,0);

            telemetry.addLine("Strafe Left");
            telemetry.update();
            for(int s=0; s < 2000; ++s){driveFunction(1, 90);}
            driveFunction(0,0);

            telemetry.addLine("Open Claw");
            telemetry.update();
            openClaw();

            telemetry.addLine("Strafe Right");
            telemetry.update();
            for(int s=0; s < 2000; ++s){driveFunction(1, 270);}
            driveFunction(0,0);

            telemetry.addLine("Moving forward to block");
            telemetry.update();
            forwardToBlock();

            telemetry.addLine("Opening Claw");
            telemetry.update();
            openClaw();

            telemetry.addLine("Moving forward a bit more");
            telemetry.update();
            for(int s=0; s < 500; ++s){driveFunction(0.5, 0);}
            driveFunction(0,0);

            telemetry.addLine("Closing Claw");
            telemetry.update();
            closeClaw();

            telemetry.addLine("Drive backwards");
            telemetry.update();
            for(int s=0; s < 1000; ++s){driveFunction(1, 180);}
            driveFunction(0,0);

            telemetry.addLine("Strafe Left");
            telemetry.update();
            for(int s=0; s < 2000; ++s){driveFunction(1, 90);}
            driveFunction(0,0);

            telemetry.addLine("Open Claw");
            telemetry.update();
            openClaw();

            telemetry.addLine("Strafe Left");
            telemetry.update();
            for(int s=0; s < 2000; ++s){driveFunction(1, 90);}
            driveFunction(0,0);

            telemetry.addLine("Closing Claw");
            telemetry.update();
            closeClaw();

            telemetry.addLine("Pivoting");
            telemetry.update();
            for(int s=0; s < 2000; ++s){rotateFunction(1, -1);}
            driveFunction(0,0);

            telemetry.addLine("Strafe Left");
            telemetry.update();
            for(int s=0; s < 500; ++s){driveFunction(1, 90);}
            driveFunction(0,0);

            telemetry.addLine("Open Claw");
            telemetry.update();
            openClaw();

            telemetry.addLine("Strafe Right");
            telemetry.update();
            for(int s=0; s < 250; ++s){driveFunction(1, 270);}
            driveFunction(0,0);

            telemetry.addLine("Drive backwards");
            telemetry.update();
            for(int s=0; s < 1300; ++s){driveFunction(1, 180);}
            driveFunction(0,0);

            while (opModeIsActive()) {
                // Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
                telemetry.addLine(" ________________________________");
                telemetry.addLine("|                                |");
                telemetry.addLine("|    Hopefully We Won!           |");
                telemetry.addLine("|         Or Andrew Messed Up    |");
                telemetry.addLine("|_____________________By_:_Aleks_|");
                telemetry.update();

            }
        }
    }


    //Functions
    private void pathClosetToWall(){
        for(int s=0; s < 500; ++s){driveFunction(0.5, 90);}
        driveFunction(0,0);
    }
    private void pathMiddleToWall(){
        for(int s=0; s < 500; ++s){driveFunction(0, 0);}
        driveFunction(0,0);
    }
    private void pathFarthestFromWall(){
        for(int s=0; s < 500; ++s){driveFunction(0.5, 270);}
        driveFunction(0,0);
    }
    private void forwardToBlock(){
        do{driveFunction(1,0);}while( frontLeftDistanceSensor.getDistance(DistanceUnit.CM) >= 5 || frontRightDistanceSensor.getDistance(DistanceUnit.CM) >= 5 );
        driveFunction(0,0);
    }

    private void driveFunction(double Power, double Angle) {
        double pureAngle = Angle - 90;
        double drive = Math.sin(pureAngle);
        double strafe = Math.cos(pureAngle);

        double roughFrontLeft  = drive + strafe;
        double roughFrontRight = drive - strafe;
        double roughBackLeft   = drive - strafe;
        double roughBackRight  = drive + strafe;

        double multiplier = 1 / (Math.abs(drive) + Math.abs(strafe));

        double pureFrontLeft  = roughFrontLeft  *  multiplier  *  Power;
        double pureFrontRight = roughFrontRight *  multiplier  *  Power;
        double pureBackLeft   = roughBackLeft   *  multiplier  *  Power;
        double pureBackRight  = roughBackRight  *  multiplier  *  Power;

        frontLeftMotor.setPower(  pureFrontLeft  );
        frontRightMotor.setPower( pureFrontRight );
        backLeftMotor.setPower(   pureBackLeft   );
        backRightMotor.setPower(  pureBackRight  );

        telemetry.addLine((
                "FL:" + frontLeftMotor.getPowerFloat()  +
                "FR:" + frontRightMotor.getPowerFloat() ));
        telemetry.addLine((
                "BL:" + backLeftMotor.getPowerFloat()   +
                "BR:" + backRightMotor.getPowerFloat()  ));
        telemetry.update();


    }

    //CHANGE ALL OF THESE TO THE FTC CHAD THING SO WE USE INCHES, NOT TIME
    private void rotateFunction(double Power, double Rotate) {
        if(Rotate == 1){
            frontLeftMotor.setPower( Power * -1);
            frontRightMotor.setPower(Power * 1 );
            backLeftMotor.setPower(  Power * -1);
            backRightMotor.setPower( Power * 1 );
        } else if(Rotate == -1){
            frontLeftMotor.setPower( Power * 1 );
            frontRightMotor.setPower(Power * -1);
            backLeftMotor.setPower(  Power * 1 );
            backRightMotor.setPower( Power * -1);
        } else {
            frontLeftMotor.setPower( Power * 0 );
            frontRightMotor.setPower(Power * 0 );
            backLeftMotor.setPower(  Power * 0 );
            backRightMotor.setPower( Power * 0 );
        }
        telemetry.addLine((
                "FL:" + frontLeftMotor.getPowerFloat()  +
                "FR:" + frontRightMotor.getPowerFloat() ));
        telemetry.addLine((
                "BL:" + backLeftMotor.getPowerFloat()   +
                "BR:" + backRightMotor.getPowerFloat()  ));
        telemetry.update();
    }
    private void openClaw(){}
    private void closeClaw(){}

    public void initVuforia() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = "AaNw5yn/////AAABmTPlCEPTXU4qsGbvT4c4pJkkfe22zDBnziTeK5i9UQn1uyHAWKWoN66qJrmWDWLBMAC/v0bdnpECUWVfAQciebhC4RSEQdvBJMoLNAA8+mqUgb7Q53oKKR267q8r4PdvMj8nQ/L8bpXQs/ohKNy2j/u+Ce8++dWdX+XynrmxMz4G9nKgp5ZPSO2NK9yJ8vCBmqJy307nfzQ/pK9n5gj2UOpe19IikYuSte5meyM5H6OR0JwnVQwN3xgTy3baT/bxz9aaK1CGkp5N1IOKg8fsx8lEXw94IVdQSI9oPjFhZKZG1wZfHt1/P97CzOcJ6CWUtnhJg2azB0doemtTwo30MIH24GNrY5LfEKkNbLZUE4gg";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
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
        //telemetry.addLine(Arrays.toString(block1Test));
        //telemetry.addLine(Arrays.toString(block2Test));
        //telemetry.addLine(Arrays.toString(block3Test));

        if(block1Test[3]<block2Test[3]&&block1Test[3]<block3Test[3]){
            if(colorSide==1){
                closeTest=true;
                blockResult = 1;
            }else{
                farTest=true;
            }
        }else if(block2Test[3]<block1Test[3]&&block2Test[3]<block3Test[3]){
            if(colorSide==1){
                centerTest=true;
                blockResult = 2;
            }else{
                centerTest=true;
            }
        }else if(block3Test[3]<block2Test[3]&&block3Test[3]<block1Test[3]){
            if(colorSide==1){
                farTest=true;
                blockResult = 3;
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