package org.firstinspires.ftc.teamcode;

//Imports

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@Autonomous(name = "DEREK - Auto v11", group = "THIS COMP")
@Disabled

public class MoveToLineAuto extends LinearOpMode {

    //Hardware Initializing
    private DcMotor frontLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backLeftMotor;
    private DcMotor backRightMotor;


    @Override
    public void runOpMode() throws InterruptedException {
        //Code to run ONE TIME after the driver hits INIT

        //Hardware Mapping
        frontLeftMotor           =  hardwareMap.dcMotor.get("frontLeft");
        frontRightMotor          =  hardwareMap.dcMotor.get("frontRight");
        backLeftMotor            =  hardwareMap.dcMotor.get("backLeft");
        backRightMotor           =  hardwareMap.dcMotor.get("backRight");

        //Hardware Initialization
        frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        //Wait for start
        telemetry.addData("Status: ", "Done");
        telemetry.update();
        waitForStart();


        if (opModeIsActive()) {
            // Code to run ONCE when the driver hits PLAY

            driveFunction(0, 180);
            sleep(15000);

            frontRightMotor.setPower(0.5);
            backRightMotor.setPower(0.5);
            frontLeftMotor.setPower(0.5);
            backLeftMotor.setPower(0.5);
            for(int s=0; s < 2000; s+=1 ){
                telemetry.addLine("Drive forwards and get block");
                telemetry.addLine(("Waiting: " + s + "/2000"));
                telemetry.addLine(("FL: " + frontLeftMotor.getPower()  ));
                telemetry.addLine(("BL: " + backLeftMotor.getPower()   ));
                telemetry.addLine(("FR: " + frontRightMotor.getPower() ));
                telemetry.addLine(("BR: " + backRightMotor.getPower()  ));
                telemetry.update();
                sleep(1);
            }

            driveFunction(0,0);

            while (opModeIsActive()) {
                // Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
                telemetry.addLine(" __________________________");
                telemetry.addLine("|                          |");
                telemetry.addLine("| Hopefully We Won!        |");
                telemetry.addLine("|   Or Divya  Messed Up    |");
                telemetry.addLine("|_______________By_:_Aleks_|");
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
    }
}