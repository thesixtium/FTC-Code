package org.firstinspires.ftc.teamcode;

//Imports
import android.graphics.Bitmap;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;


@TeleOp(name = "(USE) Motor Test v73", group = "a1")


public class MotorsTest extends LinearOpMode {


        //Hardware Initializing
        private DcMotor frontLeftMotor;
        private DcMotor frontRightMotor;
        private DcMotor backLeftMotor;
        private DcMotor backRightMotor;

        private BNO055IMU imu;
        private Orientation angles;
        private Acceleration gravity;


        //Variable Initializing

        int cpr = 1440;                                     //counts per rotation
        int gearratio = 1    ;                                      //gearing ratio
        double diameter = 4.0;                                      //wheel diameter
        double cpi = (cpr * gearratio) / (Math.PI * diameter); //counts per inch
        double bias = 1;                                     //accounts for friction, default 0.8
        double meccyBias = 1;                                      //change to adjust only strafing movement
        double conversion = cpi * bias;                               //Converts ticks to inches
        boolean exit = false;                                    //If want to exit
        boolean running = true;
        double rotate = 0;
        double strafe = 0;
        double drive = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        //Code to run ONE TIME after the driver hits INIT
        telemetry.update();

        //Variable Initialization
        telemetry.addLine("Starting Gyro...");
        telemetry.update();
        initGyro();
        sleep(100);

        for(double i = 0; i < 50; i++){
            double percent = (i / 50) *100;
            telemetry.addLine(("Gyro Test " + i + " of 50, " + percent + "% done."));
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

        frontLeftMotor            =  hardwareMap.dcMotor.get("frontLeft");
        frontRightMotor           =  hardwareMap.dcMotor.get("frontRight");
        backLeftMotor             =  hardwareMap.dcMotor.get("backLeft");
        backRightMotor            =  hardwareMap.dcMotor.get("backRight");
        sleep(50);


        //Hardware Initialization
        telemetry.addLine("Hardware Initialzation...");
        telemetry.update();

        frontLeftMotor        .setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor         .setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor       .setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor        .setDirection(DcMotorSimple.Direction.FORWARD);

        frontLeftMotor        .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor       .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor         .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor        .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        sleep(50);


        telemetry.addData("Status: ", "Done");
        telemetry.update();

        //Wait for start
        waitForStart();


        if (opModeIsActive()) {
            // Code to run ONCE when the driver hits PLAY

            telemetry.addLine("Strafing To Position...");
            telemetry.update();
            strafeToPosition(8, 1);
            stopRobot();

            telemetry.addLine("Moving To Position...");
            telemetry.update();
            moveToPosition(24, 1);
            stopRobot();


            while (opModeIsActive()) {
                // Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP

            }
        }
    }


    //Functions

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

            telemetry.addLine(("Front Left : " + frontLeftMotor.getPower()));
            telemetry.addLine(("Front Right: " + frontRightMotor.getPower()));
            telemetry.addLine(("Back  Left : " + backLeftMotor.getPower()));
            telemetry.addLine(("Back  Right: " + backRightMotor.getPower()));
            telemetry.update();

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
    public void     stopRobot         (){
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
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
        telemetry.addData("stuff", speedDirection);
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
            telemetry.addLine(("Front Left : " + frontLeftMotor.getPower()));
            telemetry.addLine(("Front Right: " + frontRightMotor.getPower()));
            telemetry.addLine(("Back  Left : " + backLeftMotor.getPower()));
            telemetry.addLine(("Back  Right: " + backRightMotor.getPower()));
            telemetry.update();
        }
        frontRightMotor.setPower(0);
        frontLeftMotor.setPower(0);
        backRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        return;
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

}