package org.firstinspires.ftc.teamcode;

//Imports
import android.graphics.Bitmap;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;


@Autonomous(name = "USE THIS AUTO - Blue Auto Block Side v79", group = "a1")


public class EdmontonBlueAutoBlockStart extends LinearOpMode {

    //Hardware Initializing
    private Servo           clawClampServo;
    private Servo           capstoneServo;
    private Servo           leftFrontClamp;
    private Servo           rightFrontClamp;

    private DcMotor         frontLeftMotor;
    private DcMotor         frontRightMotor;
    private DcMotor         backLeftMotor;
    private DcMotor         backRightMotor;

    private DcMotor         rackAndPinionMotor;
    private DcMotor         pullySystemMotor;
    private DcMotor         leftIntakeWheel;
    private DcMotor         rightIntakeWheel;

    private BNO055IMU       imu;
    private Orientation     angles;
    private Acceleration    gravity;


    //Variable Initializing
    private VuforiaLocalizer vuforia;
    boolean farTest, centerTest, closeTest;
    int testxStart = 288, testyStart = 400, testwidth = 670, testheight = 112;

    int      blockResult  =  0;                                        //Quarry Pattern (1=A, 2=B, 3=C)
    int      colorSide    =  1;                                        //DON'T TOUCH THIS
    int      cpr          =  1440;                                      //counts per rotation
    int      gearratio    =  1;                                      //gearing ratio
    double   diameter     =  4.0;                                      //wheel diameter
    double   cpi          =  (cpr * gearratio) / (Math.PI * diameter); //counts per inch
    double   bias         =  1;                                      //accounts for friction, default 0.8
    double   meccyBias    =  1;                                      //change to adjust only strafing movement
    double   conversion   =  cpi * bias;                               //Converts ticks to inches
    boolean  exit         =  false;                                    //If want to exit


    @Override
    public void runOpMode() throws InterruptedException {
        //Code to run ONE TIME after the driver hits INIT
        telemetry.update();

        //Variable Initialization
        telemetry.addLine("Starting Vuforia...");
        telemetry.update();
        initVuforia();

        telemetry.addLine("Starting Gyro...");
        telemetry.update();
        initGyro();

        for(double i = 0; i < 10; i++){
            double percent = (i / 10) *100;
            telemetry.addLine(("Gyro Test " + i + " of 10, " + percent + "% done."));
            telemetry.addLine(("Accelerometer Calibrated? : " + imu.isAccelerometerCalibrated()));
            telemetry.addLine(("Gyro Calibrated? : " + imu.isGyroCalibrated()));
            telemetry.addLine(("Magnetometer Calibrated? : " + imu.isMagnetometerCalibrated()));
            telemetry.addLine(("System Calibrated? : " + imu.isSystemCalibrated()));
            telemetry.addLine(("Status: " + imu.getSystemStatus()));
            telemetry.addLine(("Acceleration: " + imu.getAcceleration()));
            telemetry.addLine(("Angles: " + imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES)));
            telemetry.update();
            sleep(1);
        }

        //Hardware Mapping
        telemetry.addLine("Mapping hardware...");
        telemetry.update();

        clawClampServo            =  hardwareMap.servo.get("clawClamp");
        capstoneServo             =  hardwareMap.servo.get("capstoneServo");
        rightFrontClamp           =  hardwareMap.servo.get("rightFrontClamp");
        leftFrontClamp            =  hardwareMap.servo.get("leftFrontClamp");

        frontLeftMotor            =  hardwareMap.dcMotor.get("frontLeft");
        frontRightMotor           =  hardwareMap.dcMotor.get("frontRight");
        backLeftMotor             =  hardwareMap.dcMotor.get("backLeft");
        backRightMotor            =  hardwareMap.dcMotor.get("backRight");

        pullySystemMotor          =  hardwareMap.dcMotor.get("pullySystemMotor");
        rackAndPinionMotor        =  hardwareMap.dcMotor.get("rackAndPinionMotor");
        leftIntakeWheel           =  hardwareMap.dcMotor.get("leftIntakeWheel");
        rightIntakeWheel          =  hardwareMap.dcMotor.get("rightIntakeWheel");

        //Hardware Initialization
        frontLeftMotor        .setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor         .setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor       .setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor        .setDirection(DcMotorSimple.Direction.FORWARD);

        pullySystemMotor      .setDirection(DcMotorSimple.Direction.FORWARD);
        rackAndPinionMotor    .setDirection(DcMotorSimple.Direction.FORWARD);

        frontLeftMotor        .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor       .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor         .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor        .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        pullySystemMotor      .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rackAndPinionMotor    .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightIntakeWheel      .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftIntakeWheel       .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        stopRobot();

        for (int i = 0; i < 10; ++i) {
            findSkystone();
            telemetry.update();
            telemetry.addLine("Scanning...");
            telemetry.addData("Test Number: ", i);
            telemetry.addData("Block: ", blockResult);
            telemetry.update(); }

        sleep(500);
        telemetry.addData("Block Results: ", blockResult);
        telemetry.addData("Status: ", "Done");
        telemetry.update();

        //Wait for start
        waitForStart();


        if (opModeIsActive()) {
            // Code to run ONCE when the driver hits PLAY

            for (int i = 0; i < 2; ++i) {
                findSkystone();
                telemetry.update();
                telemetry.addLine("Scanning...");
                telemetry.addData("Test Number: ", i);
                telemetry.addData("Block: ", blockResult);

                telemetry.update(); }

            //Adjust position
            if(blockResult == 1) { timeStrafe(600, -0.6);}
            if(blockResult == 3) { timeStrafe(400, 0.6);}

            //Go get block
            intakePower( 1 );
            timeMove(2500, .3);

            //Back up
            intakePower( 0 );
            timeMove(1000, -.3);
            timeStrafe(300, -0.5);
            if(blockResult==3){timeStrafe(300, -0.5);};
            timeMove(4000, -0.3);
            timeMove(1000, 0.5);

            //Turn
            intakePower( 0 );
            timeTurn(450, 1);

            //Go deposit block
                intakePower(0);
                timeMove(500, 1);
                intakePower(-1);
                timeMove(1500, .6);


            //Park on line
            intakePower(-1);
            timeMove(1000, -1);

            while (opModeIsActive()) {
                // Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP


            }
        }
    }


    //Functions

    public void     initVuforia       () {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "AaNw5yn/////AAABmTPlCEPTXU4qsGbvT4c4pJkkfe22zDBnziTeK5i9UQn1uyHAWKWoN66qJrmWDWLBMAC/v0bdnpECUWVfAQciebhC4RSEQdvBJMoLNAA8+mqUgb7Q53oKKR267q8r4PdvMj8nQ/L8bpXQs/ohKNy2j/u+Ce8++dWdX+XynrmxMz4G9nKgp5ZPSO2NK9yJ8vCBmqJy307nfzQ/pK9n5gj2UOpe19IikYuSte5meyM5H6OR0JwnVQwN3xgTy3baT/bxz9aaK1CGkp5N1IOKg8fsx8lEXw94IVdQSI9oPjFhZKZG1wZfHt1/P97CzOcJ6CWUtnhJg2azB0doemtTwo30MIH24GNrY5LfEKkNbLZUE4gg";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
        vuforia.setFrameQueueCapacity(1);
        vuforia.enableConvertFrameToBitmap();
    }
    public void     initGyro          () {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "GyroCal.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        //
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }
    public void     moveToPosition    (double inches, double speed) {
        //To drive backward, simply make the inches input negative.
        //
        int move = (int) (Math.round(inches * conversion));
        //
        backLeftMotor.setTargetPosition(backLeftMotor.getCurrentPosition() + move);
        frontLeftMotor.setTargetPosition(frontLeftMotor.getCurrentPosition() + move);
        backRightMotor.setTargetPosition(backRightMotor.getCurrentPosition() + move);
        frontRightMotor.setTargetPosition(frontRightMotor.getCurrentPosition() + move);
        //
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //
        frontLeftMotor.setPower(speed);
        backLeftMotor.setPower(speed);
        frontRightMotor.setPower(speed);
        backRightMotor.setPower(speed);
        //
        while (frontLeftMotor.isBusy() && frontRightMotor.isBusy() && backLeftMotor.isBusy() && backRightMotor.isBusy()) {
            if (exit) {
                frontRightMotor.setPower(0);
                frontLeftMotor.setPower(0);
                backRightMotor.setPower(0);
                backLeftMotor.setPower(0);
                return;
            }
        }
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
        return;
    }
    public void     turnWithGyro      (double degrees, double speedDirection) {
        //Degrees should always be positive, make speedDirection negative to turn left.
        //<editor-fold desc="Initialize">
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double yaw = -angles.firstAngle;//make this negative
        telemetry.addData("Speed Direction", speedDirection);
        telemetry.addData("Yaw", yaw);
        telemetry.update();
        //
        double first;
        double second;
        //</editor-fold>
        //
        if (speedDirection > 0) {//set target positions
            //<editor-fold desc="turn right">
            if (degrees > 10) {
                first = (degrees - 10) + devertify(yaw);
                second = degrees + devertify(yaw);
            } else {
                first = devertify(yaw);
                second = degrees + devertify(yaw);
            }
            //</editor-fold>
        } else {
            //<editor-fold desc="turn left">
            if (degrees > 10) {
                first = devertify(-(degrees - 10) + devertify(yaw));
                second = devertify(-degrees + devertify(yaw));
            } else {
                first = devertify(yaw);
                second = devertify(-degrees + devertify(yaw));
            }
            //
            //</editor-fold>
        }
        //
        //<editor-fold desc="Go to position">
        Double firsta = convertify(first - 5);//175
        Double firstb = convertify(first + 5);//-175
        //
        turnWithEncoder(speedDirection);
        //
        if (Math.abs(firsta - firstb) < 11) {
            while (!(firsta < yaw && yaw < firstb) && opModeIsActive()) {//within range?
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                gravity = imu.getGravity();
                yaw = -angles.firstAngle;
                telemetry.addData("Position", yaw);
                telemetry.addData("first before", first);
                telemetry.addData("first after", convertify(first));
                telemetry.update();
            }
        } else {
            //
            while (!((firsta < yaw && yaw < 180) || (-180 < yaw && yaw < firstb)) && opModeIsActive()) {//within range?
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                gravity = imu.getGravity();
                yaw = -angles.firstAngle;
                telemetry.addData("Position", yaw);
                telemetry.addData("first before", first);
                telemetry.addData("first after", convertify(first));
                telemetry.update();
            }
        }
        //
        Double seconda = convertify(second - 5);//175
        Double secondb = convertify(second + 5);//-175
        //
        turnWithEncoder(speedDirection / 3);
        //
        if (Math.abs(seconda - secondb) < 11) {
            while (!(seconda < yaw && yaw < secondb) && opModeIsActive()) {//within range?
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                gravity = imu.getGravity();
                yaw = -angles.firstAngle;
                telemetry.addData("Position", yaw);
                telemetry.addData("second before", second);
                telemetry.addData("second after", convertify(second));
                telemetry.update();
            }
            while (!((seconda < yaw && yaw < 180) || (-180 < yaw && yaw < secondb)) && opModeIsActive()) {//within range?
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                gravity = imu.getGravity();
                yaw = -angles.firstAngle;
                telemetry.addData("Position", yaw);
                telemetry.addData("second before", second);
                telemetry.addData("second after", convertify(second));
                telemetry.update();
            }
            frontLeftMotor.setPower(0);
            frontRightMotor.setPower(0);
            backLeftMotor.setPower(0);
            backRightMotor.setPower(0);
        }
        //</editor-fold>
        //
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void     strafeToPosition  (double inches, double speed) {
        //Negative input for inches results in left strafing.
        //
        int move = (int) (Math.round(inches * cpi * meccyBias));
        //
        backLeftMotor.setTargetPosition(backLeftMotor.getCurrentPosition() - move);
        frontLeftMotor.setTargetPosition(frontLeftMotor.getCurrentPosition() + move);
        backRightMotor.setTargetPosition(backRightMotor.getCurrentPosition() + move);
        frontRightMotor.setTargetPosition(frontRightMotor.getCurrentPosition() - move);
        //
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //
        frontLeftMotor.setPower(speed);
        backLeftMotor.setPower(speed);
        frontRightMotor.setPower(speed);
        backRightMotor.setPower(speed);
        //
        while (frontLeftMotor.isBusy() && frontRightMotor.isBusy() && backLeftMotor.isBusy() && backRightMotor.isBusy()) {
        }
        frontRightMotor.setPower(0);
        frontLeftMotor.setPower(0);
        backRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        return;
    }
    public void     findSkystone      () throws InterruptedException {

        farTest = false;
        centerTest = false;
        closeTest = false;

        Bitmap bitmap = vuforia.convertFrameToBitmap(vuforia.getFrameQueue().take());
        int width = bitmap.getWidth();
        int height = bitmap.getHeight();
        int[] rawColorArray = new int[bitmap.getWidth() * bitmap.getHeight()];
        bitmap.getPixels(rawColorArray, 0, width, 0, 0, width, height);
        double[][][] pixels = new double[width][height][3];
        for (int j = 0; j < height; j++) {
            for (int i = 0; i < width; i++) {
                //telemetry.addLine("width: " + i + " height: " + j + " value: "+((rawColorArray[j*width+i])&0xFF));
                pixels[i][j][0] = ((rawColorArray[j * width + i] >> 16) & 0xFF);
                pixels[i][j][1] = ((rawColorArray[j * width + i] >> 8) & 0xFF);
                pixels[i][j][2] = ((rawColorArray[j * width + i]) & 0xFF);
            }
        }
        int increment = testwidth / 3;
        double[] block1Test = averageValues(pixels, testxStart + 10, increment - 20, testyStart, testheight);
        double[] block2Test = averageValues(pixels, testxStart + increment + 10, increment - 20, testyStart, testheight);
        double[] block3Test = averageValues(pixels, testxStart + increment * 2 + 10, increment - 20, testyStart, testheight);

        //r,g,b,yellows,blacks
        //telemetry.addLine(Arrays.toString(block1Test));
        //telemetry.addLine(Arrays.toString(block2Test));
        //telemetry.addLine(Arrays.toString(block3Test));

        if (block1Test[3] < block2Test[3] && block1Test[3] < block3Test[3]) {
            if (colorSide == 1) {
                closeTest = true;
                blockResult = 1;
            } else {
                farTest = true;
            }
        } else if (block2Test[3] < block1Test[3] && block2Test[3] < block3Test[3]) {
            if (colorSide == 1) {
                centerTest = true;
                blockResult = 2;
            } else {
                centerTest = true;
            }
        } else if (block3Test[3] < block2Test[3] && block3Test[3] < block1Test[3]) {
            if (colorSide == 1) {
                farTest = true;
                blockResult = 3;
            } else {
                closeTest = true;
            }
        }
        telemetry.addLine(("close " + closeTest) + ("   center " + centerTest) + ("    far " + farTest));
        telemetry.update();
    }
    public double[] averageValues     (double[][][] pixels, int xStart, int xLength, int yStart, int yLength) {
        double[] temp = new double[5];
        int yellows = 0;
        int blacks = 0;


        double numPixels = xLength * yLength;
        for (int j = yStart; j <= (yStart + yLength); j++) {
            for (int i = xStart; i <= (xStart + xLength); i++) {
                //telemetry.addLine("j: " + j + " i: " + i + "Values: " + pixels[i][j][0] +","+ pixels[i][j][1]+","+ pixels[i][j][2]);
                temp[0] += pixels[i][j][0];
                temp[1] += pixels[i][j][1];
                temp[2] += pixels[i][j][2];
            }
        }
        temp[0] /= numPixels;
        temp[1] /= numPixels;
        temp[2] /= numPixels;
        temp[0] = Math.round(temp[0]);
        temp[1] = Math.round(temp[1]);
        temp[2] = Math.round(temp[2]);

        for (int j = yStart; j <= (yStart + yLength); j++) {
            for (int i = xStart; i <= (xStart + xLength); i++) {
                //telemetry.addLine("j: " + j + " i: " + i + "Values: " + pixels[i][j][0] +","+ pixels[i][j][1]+","+ pixels[i][j][2]);
                if (pixels[i][j][0] > 90 && pixels[i][j][1] > 90 && pixels[i][j][2] < 120 && pixels[i][j][0] + pixels[i][j][1] > pixels[i][j][2] * 2.7) {
                    yellows++;
                }
                if (pixels[i][j][0] < 100 && pixels[i][j][1] < 100 && pixels[i][j][2] < 100 && (pixels[i][j][0] + pixels[i][j][1] + pixels[i][j][2]) / 3 < 75) {
                    blacks++;
                }
            }
        }
        temp[3] = yellows;
        temp[4] = blacks;
        return temp;
    }
    public double   devertify         (double degrees) {
        if (degrees < 0) {
            degrees = degrees + 360;
        }
        return degrees;
    }
    public double   convertify        (double degrees) {
        if (degrees > 179) {
            degrees = -(360 - degrees);
        } else if (degrees < -180) {
            degrees = 360 + degrees;
        } else if (degrees > 360) {
            degrees = degrees - 360;
        }
        return degrees;
    }
    public void     turnWithEncoder   (double input) {
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //
        frontLeftMotor.setPower(input);
        backLeftMotor.setPower(input);
        frontRightMotor.setPower(-input);
        backRightMotor.setPower(-input);
    }
    public void     intakePower (double power){
        leftIntakeWheel.setPower(-power);
        rightIntakeWheel.setPower(power);
    }
    public void     stopRobot         (){
        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);

        pullySystemMotor.setPower(0);
        rackAndPinionMotor.setPower(0);
        sleep(500);
    }
    public void     timeMove         (long time, double power){
        frontLeftMotor.setPower(power);
        backLeftMotor.setPower(power);
        frontRightMotor.setPower(power);
        backRightMotor.setPower(power);
        sleep(time);
        stopRobot();
    }
    public void     timeStrafe         (long time, double power){
        frontLeftMotor.setPower(power);
        backLeftMotor.setPower(-power);
        frontRightMotor.setPower(-power);
        backRightMotor.setPower(power);
        sleep(time);
        stopRobot();
    }
    public void     timeTurn         (long time, double power){
        frontLeftMotor.setPower(-power);
        backLeftMotor.setPower(-power);
        frontRightMotor.setPower(power);
        backRightMotor.setPower(power);
        sleep(time);
        stopRobot();
    }

}