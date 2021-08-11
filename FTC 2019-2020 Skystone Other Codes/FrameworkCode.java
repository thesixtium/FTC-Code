package org.firstinspires.ftc.teamcode;

//Imports
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.JavaUtil;



@TeleOp(name = "Framework Code (Pure Java)2", group = "Framework")
@Disabled

public class FrameworkCode extends LinearOpMode {

    //Hardware Initializing
    private Servo clawClampServo;
    private CRServo zAxisClawSpinSevo;
    private DcMotor frontLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backLeftMotor;
    private DcMotor backRightMotor;
    private DcMotor xAxisClawMoveMotor;
    private DcMotor yAxisClawMoveMotor;



    @Override
    public void runOpMode() {
        //Code to run ONE TIME after the driver hits INIT


        //Variable Initialization
        double speedMultiplier;


        //Hardware Mapping
        clawClampServo     =  hardwareMap.servo.get("clawClamp");
        zAxisClawSpinSevo  =  hardwareMap.crservo.get("zAxisClawSpin");
        frontLeftMotor     =  hardwareMap.dcMotor.get("frontLeft");
        frontRightMotor    =  hardwareMap.dcMotor.get("frontRight");
        backLeftMotor      =  hardwareMap.dcMotor.get("backLeft");
        backRightMotor     =  hardwareMap.dcMotor.get("backRight");
        xAxisClawMoveMotor =  hardwareMap.dcMotor.get("xAxisClawMove");
        yAxisClawMoveMotor =  hardwareMap.dcMotor.get("yAxisClawMove");


        //Hardware Initialization
        frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        xAxisClawMoveMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        yAxisClawMoveMotor.setDirection(DcMotorSimple.Direction.FORWARD);


        //Set variables
        speedMultiplier    =  1;


        //Wait for start
        waitForStart();



        if (opModeIsActive()) {
            // Code to run ONCE when the driver hits PLAY



            while (opModeIsActive()) {
                // Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP

                //Driving
                driveFunction(0, 0, 0);

                //Speed Multiplier
                if(this.gamepad1.y){speedMultiplier = 1;  }
                if(this.gamepad1.b){speedMultiplier = 0.5;}
                if(this.gamepad1.x){speedMultiplier = 0.5;}
                if(this.gamepad1.a){speedMultiplier = 0.2;}

                //Telemetry
                telemetry.addData("X Axis Claw Motor: ", Double.parseDouble(JavaUtil.formatNumber(xAxisClawMoveMotor.getPower(), 1)));
                telemetry.addData("Y Axis Claw Motor: ", Double.parseDouble(JavaUtil.formatNumber(yAxisClawMoveMotor.getPower(), 1)));
                telemetry.addData("Speed Multiplier: ", Double.parseDouble(JavaUtil.formatNumber(speedMultiplier, 1)));
                telemetry.update();
            }
        }
    }

    //Functions
    private void driveFunction(double Power, double Angle, double Rotate) {



    }
}