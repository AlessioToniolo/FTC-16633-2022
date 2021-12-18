package org.firstinspires.ftc.teamcode.opmode.Teleop;
import org.firstinspires.ftc.teamcode.util.BaseRobot;
import org.firstinspires.ftc.teamcode.util.helpers.Printer;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;


//things I still want to make
/*
1 - figure out the math and code to keep track of the robots orientation and make a button to orient robot maybe expiriment with rewrting turn code
2 - make intake spin in for a seconds and then out to automatically fix blocks that are stuck
3- player2 right stick button control linear slider position maybe???
4- more capstone arm automation, maybe one button for standing up and another for on the ground
5- quality of life fixes- when bucket is put into holding position intake is stopped, intake bar moves if in holding and linear slider is pressed up and doesnt break :)

 */


@TeleOp
public class Dual_Control_TeleopTank_2021 extends OpMode {
    // Use the class created to define  Robot's hardware
    BaseRobot robot = new BaseRobot();
    //dual control variable
    Gamepad curgamepad = null;
    Gamepad curgamepad2 = null;
    Gamepad player1 = null;
    Gamepad player2 = null;
    /*There are basically 4 types of controlled robot functions
    1.) the functions that player 1 always has control over
    2.) the functions player2 always ahs control over
    3.) the functions that are necessary to the robots functionality but are controlled by player 2
    4.) the functions that are unecessary to the robot's functionality but are controlled by player 1

    the player1 gamepad object controls all of the first
    the player2 gamepad object controls all of the second
    the curgamepad controls all of the third
    the curgamepad2 controls all of the fourth

    1-> drive, intake and reverse intake, capstone automatic positions, drivespeed decreaser, noramal carousels, speed cap
    2 ->  timed carousel spinners,  carosuel speed modifiers, linear slider positions
    3 -> linear sldier manual adjustments, A cycle(bucket stages)
    4 -> capstone manual control

     */
    int gamemode = 0;//0 if off 1 if driver control 2 if endgame 3 if 15 seconds left till endgame
    Printer printer = new Printer();

    //toggle variables for buttons the ones with 2 on the end are for the second gamepad
    boolean preValueA = false;
    boolean preValueB = false;
    boolean preValueB2 = false;
    boolean preValueX = false;
    boolean preValueX2 = false;
    boolean preValueY = false;
    boolean preValueY2 = false;
    boolean preValueDRight = false;
    boolean preValueDLeft = false;//left
    boolean preValueDUp = false;
    boolean preValueDDown = false;

    boolean preValueRTrigger = false;
    boolean preValueLTrigger = false;
    boolean preValueRBumper = false;
    boolean preValueRBumper2 = false;
    boolean preValueLBumper = false;
    boolean preValueLBumper2 = false;

    boolean preValueBack = false;
    boolean preValueStart = false;
    boolean preValueGuide = false;
    boolean leftStickButton = false;
    boolean rightStickButton = false;

    //variables which track bucketStates of servos and motors
    double speed = 1; //tracks speed of motors
    double curspeed = 1;//tracks the durrent speed fo the motors and is used to remember the past speed when activating a speed boost

    double intake = 0; //if 0 intake is off, if 1 intake is on -1 is intake is in reverse


    boolean redcarouselActive = false;//track if red and blue carousel are on
    boolean bluecarouselActive = false;
    final double stop = robot.stop;//time for how long carousel should be stopped for
    final double go = robot.go; // time for how long carousel should be stopped for
    double goTime = 0;//updated variable for how long carousel shoudl be run for changes based on speed of carousel
    //timers for autonmatic carousel spinning
    private ElapsedTime redStop = null;
    private ElapsedTime redGo = null;
    private ElapsedTime blueStop = null;
    private ElapsedTime blueGo = null;
    private ElapsedTime runtime  = null; //tracks elapsed time





    int intakeBar = 0;//0 if not above 1 if above

    double sliderPower = 0;// Variable for linear slider manual power
    double servoPower = 0;//variable for detecting if triggers are pressed and how much they are pressed

    int bucketState = 0; //bucketState of a toggle for bucket 0 if intake 1 if holding with bar 2 if holding without bar 3 if emptying
    int capstonebucketState = 0; // 0 if at rest 1 if picking up and 2 if capping

    // Variable of linear slider target position
    int target = 0;//the position the slider is told to run to
    double servotarget = 0;// the postion the capstone is set to in manual control

    // Constant for max number of encoder counts for stages
    int maxCounts = 2000;//maxcounts during Drier control
    final int capstonemaxcounts = 2500;//max coutns during endgame- increases because capping requires the lienar slider to rise higher





    //final ints and doubles can be mainpulated in robot class
    //slider positions
    public final int rest = 0;
    public final int low = robot.low;
    public final int middle = robot.middle;
    public final int top = robot.top;
    int sliderstate = 0; // 0 if at rest 1 if at low 2 if at middle 3 if at high 4 if at capstone- used for autonmatic slider control
    //intake bar positions
    final double over = robot.bucketover;//.475
    final double notover = robot.bucketnotover;//.35
    //bucket positions
    final double bucketintake = robot.bucketintake; //.08
    final double holding = robot.holding; //.35
    final double outtake = 1; //.9
    //max carousel speed
    final double maxcarouselspeed = robot.maxcarouselspeed; //.9
    //modifier which slows or speed up the carousel
    double carouselmodifier = 0;
    //servo positions for capstone arm
    final double capstonerest = robot.capstonerest;//.9
    final double capstoneintake = robot.capstoneintake;//unknown
    final double capstonecapping = robot.capstonecapping;// .27 doublethink is what they want you to think (run or the party will find you)
    //dual control booleans
    boolean practicemode = false; //if practice mode is true than the dual controls are kept but the timer is ignored
    boolean dualcontrols = true; //if ture dueal controls are used else all necessary functionality is switched to player 1

    @Override
    public void init() {
        // Init hardware variables
        robot.init(hardwareMap);
        player1 = gamepad1;
        player2 = gamepad2;
        // Send telemetry message to signify robot waiting
        telemetry.addData("Status: ", "Robot Ready");
        telemetry.addLine("");
        telemetry.addData("Warning: ", "Servo Moves on Initalization");

        // Set to Run without Encoder for Tele Operated
        robot.leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Reset the value of the encoder of the linear slider
        robot.slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.slider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        target = robot.slider.getCurrentPosition();
        servotarget = capstonerest;

        // Reset the position of the servo for intaking
        /*robot.slider.setTargetPosition(0);
        robot.slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);//runs the motor to the target positions
        robot.slider.setPower(1);*/
        robot.bucket.setPosition(bucketintake);
        robot.intakeBar.setPosition(notover);
        robot.capstoneArm.setPosition(capstonerest);

    }

    @Override
    public void loop() {
        // if the timer hasn't started then start it
        if (runtime == null) {
            runtime = new ElapsedTime();
        }

        else if (runtime.seconds() < 121 || practicemode)
        {
            if (runtime.seconds() < 76 && runtime.seconds() > 75) {//if 15 seconds before endgame
                robot.carousel.setPower(1);//spins the carousel when it is time to get the capstone
                redcarouselActive = true;
                telemetry.speak("GET CAPSTONE!!!!!");
                telemetry.update();
                gamemode = 3;
            }// shuts the loop off after game is over unless machmode has been turned off

            else if (runtime.seconds() > 90) gamemode = 2; // game is in autonomous

            else if (runtime.seconds() < 90) gamemode = 1; // in drvier control





            if (gamemode == 1 && !practicemode) {
                maxCounts = maxCounts;
            }
            //changes encoder count dependign on gamemode in

            if (gamemode == 2 || practicemode || gamemode == 3) {
                maxCounts = capstonemaxcounts;
            }

            // Modifiable variables for current speed calculations
            double forward;
            double turn;
            double total1;
            double total2;

            if(!dualcontrols)//if we arent using dualcontrols then witch all necessary controls to player 1
            {
                curgamepad = gamepad1;
                curgamepad2 = gamepad2;

            }
            else
            {
                curgamepad = gamepad2;
                curgamepad2 = gamepad1;
            }


            // Toggle speed cap for motors
            if (player1.back && player1.back != preValueBack) {
                if (speed == 1)
                {
                    speed = 0.5;
                    curspeed = 0.5;
                }
                else if (speed != 1)
                {
                    speed = 1;
                    curspeed = 1;
                }

            }
            preValueBack = player1.back;

            if(player1.left_stick_button)
            {
                if(speed != 1)
                {
                    curspeed = speed;
                }
                speed = 1;
            }
            else
            {
                speed = curspeed;
            }
            // Adjusting the input by the speed cap
            forward = player1.left_stick_y * speed;
            turn = player1.right_stick_x * speed;

            // Kinematics
            total1 = forward - turn;
            total2 = forward + turn;

            // Setting motor powers
            robot.leftFront.setPower(total1);
            robot.rightFront.setPower(total2);
            robot.leftRear.setPower(total1);
            robot.rightRear.setPower(total2);

            double gotime = (5.0 / 3.0) / (maxcarouselspeed + carouselmodifier);//this determines howlong the spinner must spin based on the carousel speed
            if((maxcarouselspeed + carouselmodifier) == 1)
            {
                gotime = 1.5;
            }
            // Toggle intake
            if (player1.dpad_right && player1.dpad_right != preValueDRight)
            {


                if (intake == 0 || intake == -1) {
                    intake = 1;

                }
                else {
                    intake = 0;

                }
                robot.intake.setPower(intake);


            }
            preValueDRight = player1.dpad_right;

            //toggle intake backwards
            if (player1.dpad_left && player1.dpad_left != preValueDLeft) {

                intake = -1;
                robot.intake.setPower(intake);
            }

            if(player2.dpad_up && player2.dpad_up != preValueDUp)
            {
                if(intakeBar == 0) {
                    if (sliderstate == 0 || sliderstate == 1 || sliderstate == 2) {
                        sliderstate = 3;
                        target = top;
                    } else if (sliderstate == 3) {
                        sliderstate = 4;
                        target = 2500;
                    }
                    robot.slider.setTargetPosition(target);
                    robot.slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);//runs the motor to the target positions
                    robot.slider.setPower(1);

                }
            }
            preValueDUp = player2.dpad_up;

            if(player2.dpad_down && player2.dpad_down != preValueDDown)
            {
                if(intakeBar == 0)
                {
                    if(sliderstate == 4)
                    {
                        sliderstate = 3;
                        target = top;
                    }
                    else if(sliderstate== 3)
                    {
                        sliderstate = 2;
                        target = middle;
                    }
                    else if(sliderstate == 2)
                    {
                        sliderstate = 1;
                        target = low;
                    }
                    else if(sliderstate == 1)
                    {
                        sliderstate = 0;
                        target = 0;
                    }
                    robot.slider.setTargetPosition(target);
                    robot.slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);//runs the motor to the target positions
                    robot.slider.setPower(1);

                }
            }
            preValueDDown = player2.dpad_down;


            //toggles wheel direction which spins red carousel
            if (player1.b && player1.b != preValueB) {
                redGo = null;
                redStop = null;
                blueStop = null;
                blueGo = null;
                if (!redcarouselActive) {
                    robot.carousel.setPower(-1 * (maxcarouselspeed + carouselmodifier));
                    redcarouselActive = true;
                } else {
                    robot.carousel.setPower(0);
                    redcarouselActive = false;
                }
            }

            preValueB = player1.b;

            //activates carousel toggling on and off on a loop
            if ((player2.b && player2.b != preValueB2) || (redGo != null && redGo.seconds() > gotime && redcarouselActive) || (redStop != null && redStop.seconds() > stop && !redcarouselActive)) //if button is pressed or timer for go and stop reaches the limit
            {
                if (gamemode == 2 || practicemode) {// ativates the timed spinner when pressed and in endgame or if not in matchmode
                    if (!redcarouselActive) {
                        robot.carousel.setPower(-1 * (maxcarouselspeed + carouselmodifier));
                        redcarouselActive = true;
                        redGo = new ElapsedTime();
                    } else {
                        robot.carousel.setPower(0);
                        redcarouselActive = false;
                        redStop = new ElapsedTime();
                    }
                }
            }
            preValueB2 = player2.b;

            //blue carousel
            if (player1.x && player1.x != preValueX) {
                blueStop = null;
                blueGo = null;
                redGo = null;
                redStop = null;
                if (!bluecarouselActive) {
                    robot.carousel.setPower(maxcarouselspeed + carouselmodifier);
                    bluecarouselActive = true;
                } else {
                    robot.carousel.setPower(0);
                    bluecarouselActive = false;
                }

            }

            preValueX = player1.x;

            if ((player2.x && player2.x != preValueX2) || (blueGo != null && blueGo.seconds() > go && bluecarouselActive) || (blueStop != null && blueStop.seconds() > stop && !bluecarouselActive)) //if button is pressed or timer for go and stop reaches the limit
            {
                if (gamemode == 2 || practicemode) {// ativates the timed spinner when pressed and in endgame or if not in matchmode
                    if (!bluecarouselActive) {
                        robot.carousel.setPower(1 * (maxcarouselspeed + carouselmodifier));
                        bluecarouselActive = true;
                        blueGo = new ElapsedTime();
                    } else {
                        robot.carousel.setPower(0);
                        bluecarouselActive = false;
                        blueStop = new ElapsedTime();
                    }
                }
            }
            preValueX2 = player2.x;


            // Run linear slider (manually then set hold position)

            //is operated by the snd player unless not in matchmode
            sliderPower = curgamepad.right_trigger - curgamepad.left_trigger;
            if (Math.abs(sliderPower) > 0 && intakeBar == 0) {
                target += sliderPower * 10;//adds 10 to the target
                if (target > maxCounts) {
                    target = maxCounts;//if the target is greater than the maxcounts than the target = max counts
                }
                else if (target < 0) {
                    target = 0;
                }

                robot.slider.setTargetPosition(target);
                robot.slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);//runs the motor to the target positions
                robot.slider.setPower(1);
            }

            //is operated by gamepad 1 unless not in matchmode
            servoPower = curgamepad2.right_trigger - curgamepad2.left_trigger;
            if (Math.abs(servoPower) > 0) { //if the triggers are pressed and in endgame or not in matchmode
                servotarget += servoPower / 500;//adds or subtracts between .01 and 1 to the target
                if (servotarget >= .9) {
                    servotarget = .9;//if the target is greater than  1 than the servo target = 1
                } else if (servotarget <= .24) {
                    servotarget = .24;
                }

                robot.capstoneArm.setPosition(servotarget);
            }


            // Operate bucket deposit servo and is operated by player2 unless not in match mode
            if (curgamepad.a && curgamepad.a != preValueA) {
                //bucketState 0 = intake position .08
                //bucketState 1 = holding position .35
                //bucketState 3 = dropping position .9

                if (bucketState == 0)//if in intake -  set  to holding positions and move bar to over
                {
                    if(robot.slider.getCurrentPosition() < 100)
                    {
                        robot.intakeBar.setPosition(over);
                        intakeBar = 1;
                    }
                    bucketState = 1;
                        /*
                        if (robot.slider.getCurrentPosition() < 100)// as long as the bucket is beneath a certain points bring the intake bar over
                        {

                            robot.intakeBar.setPosition(.35);
                            intakeBar = 1;//change intake bar variable

                        }
                        */
                } else if (bucketState == 3)// if we are in otutake position set back to intake
                {
                    bucketState = 0;
                } else if (bucketState == 2)// if we are in holding without bar position set to outtake
                {

                    bucketState = 3;
                } else if (bucketState == 1)//if we are in holding set to holding without bar
                {

                    bucketState = 2;

                    robot.intakeBar.setPosition(notover);//moves bar back and changes variable
                    intakeBar = 0;

                }

                if (bucketState == 0)//sets to positions this is intake
                {
                    robot.slider.setTargetPosition(0);
                    robot.slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.slider.setPower(1);
                    robot.bucket.setPosition(.08);
                    robot.capstoneArm.setPosition(capstonerest);
                    capstonebucketState = 0;
                    bucketState = 0;
                    sliderstate = 0;
                    target = 0;
                } else if (bucketState == 1)//this is holding
                {
                    robot.bucket.setPosition(holding);
                    intake = 0;

                    robot.intake.setPower(intake);

                } else if (bucketState == 3)//this is outtake
                {
                    robot.bucket.setPosition(outtake);

                }

                // BUTTON A SETS SERVO FOR DEPOSITING
            }
            preValueA = curgamepad.a;


            //right bumper toggle for capstone arm
            if (player1.right_bumper && player1.right_bumper != preValueRBumper) {

                if (capstonebucketState == 0) {
                    robot.capstoneArm.setPosition(capstoneintake);
                    servotarget = capstoneintake;
                    capstonebucketState = 1;
                } else if (capstonebucketState == 1) {
                    robot.capstoneArm.setPosition(capstonecapping);
                    servotarget = capstonecapping;
                    capstonebucketState = 0;
                }

            }
            preValueRBumper = player1.right_bumper;
            if (player1.left_bumper && player1.left_bumper != preValueLBumper) {
                if(speed <= .1)
                {
                    speed = .1;
                    curspeed = speed;
                }
                else{
                    speed = speed - .25;
                    curspeed = speed;
                }

            }
            preValueLBumper = player1.left_bumper;
            if (player2.right_bumper && player2.right_bumper != preValueRBumper2) {
                if ((carouselmodifier + maxcarouselspeed) >= 1) {
                    carouselmodifier = .1;
                } else {
                    carouselmodifier += 0.1;
                }

            }
            preValueRBumper2 = player2.right_bumper;

            if (player2.left_bumper && player2.left_bumper != preValueLBumper2) {
                if ((maxcarouselspeed + carouselmodifier) <= 0) {
                    carouselmodifier = -.9;
                } else {
                    carouselmodifier -= .1;
                }

            }
            preValueLBumper2 = player2.left_bumper;
            //left bumper sets bucket and slider to intake position
            if ((player1.y && player1.y != preValueY) || (player2.y && player2.y != preValueY2)) {
                robot.slider.setTargetPosition(0);
                robot.slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.slider.setPower(1);
                robot.bucket.setPosition(.08);
                robot.capstoneArm.setPosition(capstonerest);
                capstonebucketState = 0;
                bucketState = 0;
                sliderstate = 0;
                target = 0;

            }
            preValueY = player1.y;
            preValueY2 = player2.y;



            //currently changes gamepad in control

            // if teh center button is presed than matchmode is turned off if matchmode is off all essentail controls go to player1
            if (player1.guide && player1.guide != preValueGuide) {
                if (dualcontrols) {
                    dualcontrols = false;
                } else if (!dualcontrols) {
                    dualcontrols = true;
                }
            }
            preValueGuide = player1.guide;

            if(player1.start && player1.start != preValueStart)
            {
                if(practicemode)
                {
                    practicemode = false;
                }
                else if (!practicemode && runtime.seconds() > 2.0)
                {
                    practicemode = true;
                }
            }
            preValueStart = player1.start;
            ArrayList<String> output = new ArrayList<>() ;

            if (practicemode) {
                //OUTPUTTING bucketStateS OF VARIABLES
                output.add("/////STATES/////");
                output.add("Intake:" + intake);
                output.add("bucket:" + bucketState);
                output.add("Maxcounts:  " +  maxCounts);
                output.add("SliderState:" +  sliderstate);
                output.add("Slider Position is " +  robot.slider.getCurrentPosition()  + " and target of the slider is" + target);
                output.add("Capstone:  " + servotarget);
                //OUTPUTTING SPEED OF MOTORS
                output.add("/////SPEEDS/////");
                output.add("Drive Speed:" + speed);
                output.add("Carousel speed = " + maxcarouselspeed + carouselmodifier);
                output.add("Carousel speed modifier = " + carouselmodifier);
                //OTHER DATA
                output.add("//////OTHER DATA/////");
                output.add("Time" + runtime.seconds());
                String gameModeTitle;
                if(gamemode == 1) gameModeTitle = "DRIVER CONTROL";
                else if(gamemode == 2) gameModeTitle = "GET CAPSTONE TIME";
                else gameModeTitle = "ENDGAME";
                output.add("Game Portion" +  gamemode + gameModeTitle);
                output.add("PracticeMode:  " +  practicemode);
                output.add("DualControlsActive:  " +  dualcontrols);
            }
            else {
                output.add("Carousel speed = " +  maxcarouselspeed + carouselmodifier);
                output.add("Time" + runtime.seconds());
                output.add("Game Portion" +  gamemode);
                output.add("DualControls" + dualcontrols);
                output.add("practicemode" + practicemode);

            }
            printer.printLines(output);
        }
        else if(runtime.seconds() > 121)
        {
            robot.carousel.setPower(0);
            robot.intake.setPower(0);
        }
        //once time is up
    }
}