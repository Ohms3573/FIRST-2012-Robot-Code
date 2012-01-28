/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package edu.wpi.first.wpilibj.templates;

import com.sun.squawk.util.Arrays;
import com.sun.squawk.util.Comparer;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.camera.*;
import edu.wpi.first.wpilibj.image.*;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the SimpleRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends SimpleRobot {
    // Ports 
    // Jaguars
    private final int fLeftMotorJaguarPort = 1;
    private final int rLeftMotorJaguarPort = 2;
    private final int fRightMotorJaguarPort = 3;
    private final int rRightMotorJaguarPort = 4;
    private final int ballConveyorJaguarPort = 5;
    
    // Victors
    private final int gunLeftVictorPort = 7;
    private final int gunRightVictorPort = 7;
    
    // Relays
    private final int turretTurnRelayPort = 1;
    private final int turretElevationRelayPort = 2;
    private final int conveyorTiltRelayPort = 3;
    private final int BridgingArmRelayPort = 4;
    
    private final static boolean ON = true;
    private final static boolean OFF = false;    
    
    
    // Hardware
    Joystick driverStick = new Joystick(1);
    Joystick gunnerStick = new Joystick(2);
    
    RobotDrive drive = new RobotDrive(fLeftMotorJaguarPort, rLeftMotorJaguarPort, 
            fRightMotorJaguarPort, rRightMotorJaguarPort);
    
    Jaguar ballConveyor = new Jaguar(ballConveyorJaguarPort);
    
    Victor gunLeft = new Victor(gunLeftVictorPort);
    Victor gunRight = new Victor(gunRightVictorPort);
    
    Relay turretTurn = new Relay(turretTurnRelayPort, Relay.Direction.kBoth);
    Relay turretElevation = new Relay(turretElevationRelayPort, Relay.Direction.kBoth);
    Relay conveyorTilt = new Relay(conveyorTiltRelayPort, Relay.Direction.kBoth);
    Relay BridgingArm = new Relay(BridgingArmRelayPort, Relay.Direction.kBoth);
        
    AxisCamera camera;
    private ColorImage colorImage;
    private ParticleAnalysisReport s_particles[];
    private final ParticleComparer particleComparer = new ParticleComparer();
    
    // Speeds
    private final static double CONVEYOR_BELT_SPEED = 1.0;

    public void robotInit(){
        camera = AxisCamera.getInstance();
    }
    
    public void autonomous() {
        drive.drive(0.5, 0.0); // drive 50% forward speed with 0% turn
        Timer.delay(0.5); // wait 2 seconds
        drive.drive(0.0, 1.0); // drive 0% forward speed with 100% turn
        Timer.delay(0.5); // wait for turn to complete
        drive.drive(.75, 0.0); // drive 75% forward speed with 0% turn 
        Timer.delay(0.5);
        drive.drive(0.0,0.0);
    }

    public void operatorControl() {
        ballConveyor(ON);
        while(isOperatorControl() && isEnabled()){
            drive.arcadeDrive(driverStick); // drive with the joysticks
            Timer.delay(0.005);
            
            
            if(gunnerStick.getButton(Joystick.ButtonType.kTop)){
                if(ballConveyor.get() == 0){
                    ballConveyor.set(1.0);
                }
                else if(ballConveyor.get() == 1.0){
                    ballConveyor.set(0.0);
                }
            }
        }
    }
    
////////////////////////////////////////////////////////////////////////
//  __  __     _              ___         _           _ _             //
// |  \/  |___| |_ ___ _ _   / __|___ _ _| |_ _ _ ___| | |___ _ _ ___ //
// | |\/| / _ \  _/ _ \ '_| | (__/ _ \ ' \  _| '_/ _ \ | / -_) '_(_-< //
// |_|  |_\___/\__\___/_|    \___\___/_||_\__|_| \___/_|_\___|_| /__/ //
//                                                                    //
//////////////////////////////////////////////////////////////////////// 
                                                                  
    public void ballConveyor(boolean onOff) {
        if(onOff) {
            ballConveyor.set(CONVEYOR_BELT_SPEED);
        }
        else {
            ballConveyor.set(0.0);
        }
    }

    public void tiltConveyor() {
        conveyorTilt
    }
    
    public void interpretDriverJoystick() {
        if(driverStick.getButton(Joystick.ButtonType.kNumButton)){
            alignTurret(1);
        }
    }
    
    public void interpretGunnerJoystick() {
        
    }
    
    private void alignTurret(int i) {
        double[][] blobs = getCameraInput();
        double locationX = blobs[i][0];
        double locationY = blobs[i][0];
        while(Math.abs(locationX)>=.25){
            if(locationX > 0)
                turretTurn.set(Relay.Value.kForward);
            else turretTurn.set(Relay.Value.kReverse);
            Timer.delay(0.25);
            blobs = getCameraInput();
            locationX = blobs[i][0];
        }
        turretTurn.set(Relay.Value.kOff);
        while(Math.abs(locationY)>=.25){
            if(locationX > 0)
                turretElevation.set(Relay.Value.kForward);
            else turretElevation.set(Relay.Value.kReverse);
            Timer.delay(0.25);
            blobs = getCameraInput();
            locationY = blobs[i][0];
        }
        turretElevation.set(Relay.Value.kOff);
        
        //blob area to distance conversion
        
        //distance to turret angle increase conversion
    }
    
    /**
     * 
     * @return
     */
    public double[][] getCameraInput(){ 
        //returns all possible targets center of mass x, y, degreesoff, and area
        
        double[][] cameraInfo = {};
        if (camera.freshImage()) {	    // check if there is a new image
            try {
                colorImage = camera.getImage(); // get the image from the camera

                /**
                 * The color threshold operation returns a bitmap (BinaryImage) where pixels are present
                 * when the corresponding pixels in the source (HSL) image are in the specified
                 * range of H, S, and L values.
                 */
                BinaryImage binImage = colorImage.thresholdHSL(242, 255, 36, 255, 25, 255);

                /**
                 * Find blobs (groupings) of pixels that were identified in the color threshold operation
                 */
                s_particles = binImage.getOrderedParticleAnalysisReports();

                /**
                 * Free the underlying color and binary images.
                 * You must do this since the image is actually stored
                 * as a C++ data structure in the underlying implementation to maximize processing speed.
                 */
                colorImage.free();
                binImage.free();

                if (s_particles.length > 0) {
                    /**
                     * sort the particles using the custom comparitor class (see below)
                     */
                    Arrays.sort(s_particles, particleComparer);

                    for (int i = 0; i < s_particles.length; i++) {
                        ParticleAnalysisReport circ = s_particles[i];

                        /**
                         * Compute the number of degrees off center based on the camera image size
                         */
                        double degreesOff = -((54.0 / 640.0) * ((circ.imageWidth / 2.0) - circ.center_mass_x));
                        
                        cameraInfo[i][0]=circ.center_mass_x;
                        cameraInfo[i][1]=circ.center_mass_y;
                        cameraInfo[i][2]=degreesOff;
                        cameraInfo[i][3]=circ.particleArea;
                    }
                }
                Timer.delay(4);
            } catch (AxisCameraException ex) {
                ex.printStackTrace();
            } catch (NIVisionException ex) {
                ex.printStackTrace();
            }
        }
        return cameraInfo;
    }
    
    class ParticleComparer implements Comparer {

        public int compare(ParticleAnalysisReport p1, ParticleAnalysisReport p2) {
            float p1Ratio = p1.boundingRectWidth / p1.boundingRectHeight;
            float p2Ratio = p2.boundingRectWidth / p2.boundingRectHeight;

            if (Math.abs(p1Ratio - p2Ratio) < 0.1) {
                return -(Math.abs((p1.imageWidth / 2) - p1.center_mass_x))
                        - Math.abs(((p2.imageWidth / 2) - p2.center_mass_x));
            } else {
                if (Math.abs(p1Ratio - 1) < Math.abs(p2Ratio - 1)) {
                    return 1;
                } else {
                    return -1;
                }
            }
        }

	// overloaded method because the comparitor uses Objects (not Particles)
        public int compare(Object o1, Object o2) {
            return compare((ParticleAnalysisReport) o1, (ParticleAnalysisReport) o2);
        }
    }

    public class Claw extends Subsystem {
        Victor motor;

        public Claw() {
            motor = new Victor(7);
        }

        public void initDefaultCommand() {
            setDefaultCommand(new ClawDoNothing());
        }

        public void move(double speed) {
            motor.set(speed);
        }
    }
}
