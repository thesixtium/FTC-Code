package org.firstinspires.ftc.teamcode;

//Imports

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@Autonomous(name = "DEREK - Better Auto v116", group = "THIS COMP")
@Disabled

public class GetBlockAndMoveToLineAuto extends LinearOpMode {

    //Hardware Initializing
    private DcMotor frontLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backLeftMotor;
    private DcMotor backRightMotor;
    private DcMotor leftIntakeWheel;
    private DcMotor rightIntakeWheel;


    @Override
    public void runOpMode() throws InterruptedException {
        //Code to run ONE TIME after the driver hits INIT

        //Hardware Mapping
        frontLeftMotor           =   hardwareMap.dcMotor.get("frontLeft");
        frontRightMotor          =   hardwareMap.dcMotor.get("frontRight");
        backLeftMotor            =   hardwareMap.dcMotor.get("backLeft");
        backRightMotor           =   hardwareMap.dcMotor.get("backRight");
        leftIntakeWheel          =   hardwareMap.dcMotor.get("LeftIntakeWheel");
        rightIntakeWheel         =   hardwareMap.dcMotor.get("RightIntakeWheel");


        //Hardware Initialization
        frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftIntakeWheel.setDirection(DcMotorSimple.Direction.FORWARD);
        rightIntakeWheel.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftIntakeWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightIntakeWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        //Wait for start
        telemetry.addData("Status: ", "Done");
        telemetry.update();
        waitForStart();


        if (opModeIsActive()) {
            // Code to run ONCE when the driver hits PLAY

            frontRightMotor.setPower(0.5);
            backRightMotor.setPower(0.5);
            frontLeftMotor.setPower(0.5);
            backLeftMotor.setPower(0.5);
            leftIntakeWheel.setPower(1);
            rightIntakeWheel.setPower(1);
            for(int s=0; s < 2000; s+=1 ){
                telemetry.addLine("Drive forwards and get block");
                telemetry.addLine(("Waiting: " + s + "/1500"));
                telemetry.addLine(("FL: " + frontLeftMotor.getPower()  ));
                telemetry.addLine(("BL: " + backLeftMotor.getPower()   ));
                telemetry.addLine(("FR: " + frontRightMotor.getPower() ));
                telemetry.addLine(("BR: " + backRightMotor.getPower()  ));
                telemetry.update();
                sleep(1);
            }

            frontRightMotor.setPower(-0.5);
            backRightMotor.setPower(-0.5);
            frontLeftMotor.setPower(-0.5);
            backLeftMotor.setPower(-0.5);
            leftIntakeWheel.setPower(0.0);
            rightIntakeWheel.setPower(0);
            for(int s=0; s < 3000; s+=1 ){
                telemetry.addLine("Drive to wall with block");
                telemetry.addLine(("Waiting: " + s + "/1500"));
                telemetry.addLine(("FL: " + frontLeftMotor.getPower()  ));
                telemetry.addLine(("BL: " + backLeftMotor.getPower()   ));
                telemetry.addLine(("FR: " + frontRightMotor.getPower() ));
                telemetry.addLine(("BR: " + backRightMotor.getPower()  ));
                telemetry.update();
                sleep(1);
            }

            frontRightMotor.setPower(-.75);
            backRightMotor.setPower(-.75);
            frontLeftMotor.setPower(.75);
            backLeftMotor.setPower(0.75);
            leftIntakeWheel.setPower(0);
            rightIntakeWheel.setPower(0);
            sleep(1100);

            frontRightMotor.setPower(0.5);
            backRightMotor.setPower(0.5);
            frontLeftMotor.setPower(0.5);
            backLeftMotor.setPower(0.5);
            leftIntakeWheel.setPower(-1);
            rightIntakeWheel.setPower(-1);
            sleep(4500);

            frontRightMotor.setPower(-0.5);
            backRightMotor.setPower(-0.5);
            frontLeftMotor.setPower(-0.5);
            backLeftMotor.setPower(-0.5);
            leftIntakeWheel.setPower(-1);
            rightIntakeWheel.setPower(-1);
            sleep(1000);


            driveFunction(0,0);
            leftIntakeWheel.setPower(0);
            rightIntakeWheel.setPower(0);

            while (opModeIsActive()) {
                // Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
                telemetry.addLine(" __________________________");
                telemetry.addLine("|--------------------------|");
                telemetry.addLine("|-Hopefully We Won!--------|");
                telemetry.addLine("|---Or-Divya--Messed-Up----|");
                telemetry.addLine("|_______________By_:_Aleks-|");
                telemetry.update();

            }
        }
    }


    //Functions
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
}