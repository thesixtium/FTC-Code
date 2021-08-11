package org.firstinspires.ftc.teamcode;

//Imports

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


@Autonomous(name = "(USE) Edmonton Move To Line v3", group = "a1")


public class EdmontonToLine extends LinearOpMode {

    //Hardware Initializing
    private DcMotor         frontLeftMotor;
    private DcMotor         frontRightMotor;
    private DcMotor         backLeftMotor;
    private DcMotor         backRightMotor;
    private DcMotor         leftIntakeWheel;
    private DcMotor         rightIntakeWheel;
    private DcMotor         pullySystemMotor;
    private DcMotor         rackAndPinionMotor;

    private Servo           clawClampServo;
    private Servo           capstoneServo;
    private Servo           rightFrontClamp;
    private Servo           leftFrontClamp;


    //Variable Initializing
    double   rotate       =  0     ;
    double   strafe       =  0     ;
    double   drive        =  0     ;
    boolean  slowDrive    =  false ;
    boolean  intake       =  false ;
    boolean  outtake      =  false ;

    @Override
    public void runOpMode() throws InterruptedException {
        //Code to run ONE TIME after the driver hits INIT
        telemetry.update();

        //Hardware Mapping
        telemetry.addLine("Mapping hardware...");
        telemetry.update();

        frontLeftMotor            =  hardwareMap.dcMotor.get("frontLeft");
        frontRightMotor           =  hardwareMap.dcMotor.get("frontRight");
        backLeftMotor             =  hardwareMap.dcMotor.get("backLeft");
        backRightMotor            =  hardwareMap.dcMotor.get("backRight");
        leftIntakeWheel           =  hardwareMap.dcMotor.get("leftIntakeWheel");
        rightIntakeWheel          =  hardwareMap.dcMotor.get("rightIntakeWheel");
        pullySystemMotor          =  hardwareMap.dcMotor.get("pullySystemMotor");
        rackAndPinionMotor        =  hardwareMap.dcMotor.get("rackAndPinionMotor");

        clawClampServo            =  hardwareMap.servo.get("clawClamp");
        capstoneServo             =  hardwareMap.servo.get("capstoneServo");
        rightFrontClamp           =  hardwareMap.servo.get("rightFrontClamp");
        leftFrontClamp            =  hardwareMap.servo.get("leftFrontClamp");


        sleep(50);


        //Hardware Initialization
        telemetry.addLine("Hardware Initialzation...");
        telemetry.update();

        frontLeftMotor        .setDirection(DcMotorSimple.Direction.FORWARD);
        backLeftMotor         .setDirection(DcMotorSimple.Direction.FORWARD);
        frontRightMotor       .setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor        .setDirection(DcMotorSimple.Direction.REVERSE);
        leftIntakeWheel       .setDirection(DcMotorSimple.Direction.FORWARD);
        rightIntakeWheel      .setDirection(DcMotorSimple.Direction.REVERSE);
        rackAndPinionMotor    .setDirection(DcMotorSimple.Direction.FORWARD);
        pullySystemMotor      .setDirection(DcMotorSimple.Direction.FORWARD);

        frontLeftMotor        .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor       .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor         .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor        .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftIntakeWheel       .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightIntakeWheel      .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rackAndPinionMotor    .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pullySystemMotor      .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        clawClampServo  .scaleRange( 0,  1  );
        capstoneServo   .scaleRange( 0, 180 );
        leftFrontClamp  .scaleRange( 0, 180 );
        rightFrontClamp .scaleRange( 0, 180 );

        sleep(100);


        telemetry.addData("Status: ", "Done");
        telemetry.update();

        //Wait for start
        waitForStart();


        if (opModeIsActive()) {
            // Code to run ONCE when the driver hits PLAY
            while (opModeIsActive()) {
                // Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP

                sleep(15000);

                timeMove(1500, -0.5);



            }
        }
    }


    //Functions

    public void     stopRobot         (){
        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);

        pullySystemMotor.setPower(0);
        rackAndPinionMotor.setPower(0);
        sleep(100);
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


}