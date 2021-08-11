package org.firstinspires.ftc.teamcode;

//Imports

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;


@TeleOp(name = "(USE) Edmonton Tele Op v68", group = "a1")


public class EdmontonTeleOp extends LinearOpMode {

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

            driving();
            intake();
            liftMechanism();
            clawFunction();

            telementryDashboard();



            }
        }
    }


    //Functions

    public void driving(){
        if( this.gamepad1.left_stick_x != 0 || this.gamepad1.left_stick_y != 0 || this.gamepad1.right_stick_x != 0 ) {

            slowDrive = false;

            drive = this.gamepad1.left_stick_y;
            strafe = this.gamepad1.left_stick_x;
            rotate = this.gamepad1.right_stick_x;

            frontLeftMotor.setPower(drive - strafe - rotate);
            frontRightMotor.setPower(drive + strafe + rotate);
            backLeftMotor.setPower(drive + strafe - rotate);
            backRightMotor.setPower(drive - strafe + rotate);

        } else {

            double slowPower = 0.25;
            slowDrive = true;

            if(this.gamepad1.dpad_up){
                frontLeftMotor  .setPower( ( frontLeftMotor.getPower()  + slowPower ) );
                frontRightMotor .setPower( ( frontRightMotor.getPower() + slowPower ) );
                backLeftMotor   .setPower( ( backLeftMotor.getPower()   + slowPower ) );
                backRightMotor  .setPower( ( backRightMotor.getPower()  + slowPower ) );
            } else if(this.gamepad1.dpad_down){
                frontLeftMotor  .setPower( ( frontLeftMotor.getPower()  - slowPower ) );
                frontRightMotor .setPower( ( frontRightMotor.getPower() - slowPower ) );
                backLeftMotor   .setPower( ( backLeftMotor.getPower()   - slowPower ) );
                backRightMotor  .setPower( ( backRightMotor.getPower()  - slowPower ) );
            } else if(this.gamepad1.dpad_left){
                frontLeftMotor  .setPower( ( frontLeftMotor.getPower()  - slowPower ) );
                frontRightMotor .setPower( ( frontRightMotor.getPower() + slowPower ) );
                backLeftMotor   .setPower( ( backLeftMotor.getPower()   + slowPower ) );
                backRightMotor  .setPower( ( backRightMotor.getPower()  - slowPower ) );
            } else if(this.gamepad1.dpad_right){
                frontLeftMotor  .setPower( ( frontLeftMotor.getPower()  + slowPower ) );
                frontRightMotor .setPower( ( frontRightMotor.getPower() - slowPower ) );
                backLeftMotor   .setPower( ( backLeftMotor.getPower()   - slowPower ) );
                backRightMotor  .setPower( ( backRightMotor.getPower()  + slowPower ) );
            } else {
                frontLeftMotor  .setPower( 0 );
                frontRightMotor .setPower( 0 );
                backLeftMotor   .setPower( 0 );
                backRightMotor  .setPower( 0 );
            }


        }
    }
    public void intake(){

        if( intake && !outtake ) {
            leftIntakeWheel.setPower(1);
            rightIntakeWheel.setPower(1);
        } else if( !intake && outtake ) {
            leftIntakeWheel.setPower(-1);
            rightIntakeWheel.setPower(-1);
        } else if( !intake && !outtake ) {
            leftIntakeWheel.setPower(0);
            rightIntakeWheel.setPower(0);
        } else  {
            leftIntakeWheel.setPower(0);
            rightIntakeWheel.setPower(0);
        }

        leftIntakeWheel   .setPower( this.gamepad1.left_trigger - this.gamepad1.right_trigger );
        rightIntakeWheel  .setPower( this.gamepad1.left_trigger - this.gamepad1.right_trigger );

        if (this.gamepad1.right_bumper){
            leftFrontClamp  .setPosition(  0  );
            rightFrontClamp .setPosition( 180 );
        }
        if (this.gamepad1.left_bumper){
            leftFrontClamp  .setPosition(  0  );
            rightFrontClamp .setPosition( 180 );
        }

    }
    public void liftMechanism(){

        pullySystemMotor   .setPower( this.gamepad2.right_stick_y * -0.3 );
        rackAndPinionMotor .setPower( this.gamepad2.left_stick_y  *  0.4 );

    }
    public void clawFunction(){

        if (this.gamepad2.right_bumper         ){ clawClampServo .setPosition(  0.5 ); }
        if (this.gamepad2.left_bumper          ){ clawClampServo .setPosition(  0.65 ); }

        if (this.gamepad2.x ){ capstoneServo  .setPosition(  0  ); }
        if (this.gamepad2.y ){ capstoneServo  .setPosition( 180 ); }

        if (this.gamepad2.a ){ leftFrontClamp  .setPosition(  0  ); rightFrontClamp  .setPosition(  0  );  }
        if (this.gamepad2.b ){ leftFrontClamp  .setPosition( 180 ); rightFrontClamp  .setPosition( 180 );  }


    }

    public void telementryDashboard(){
        telemetry.addLine( ( "TELEMENTRY DASHBOARD" ) );
        telemetry.addLine( ("Slow Drive Mode: " + slowDrive));
        telemetry.addLine( ("Front Left : " + frontLeftMotor.getPower() ) );
        telemetry.addLine( ("Front Right: " + frontRightMotor.getPower() ) );
        telemetry.addLine( ("Back  Left : " + backLeftMotor.getPower() ) );
        telemetry.addLine( ("Back  Right: " + backRightMotor.getPower() ) );
        telemetry.addLine( ("Left: " + leftFrontClamp.getPosition() + "     Right: " + rightFrontClamp.getPosition() ));
        telemetry.addLine( ("Claw: " + clawClampServo.getPosition() + "     Capstone: " + capstoneServo.getPosition() ));
        telemetry.update();
    }
}