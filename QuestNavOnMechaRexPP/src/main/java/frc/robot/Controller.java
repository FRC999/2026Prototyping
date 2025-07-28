package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Constants.OIContants.ControllerDevice;
import frc.robot.Constants.OIContants.ControllerDeviceType;

public class Controller extends Joystick {

    ControllerDevice cd;
    ControllerDeviceType cdt;
    double dx; // X deadband for performance
    double dy; // Y deadband for performance
    double dm; // Omega deadband for performance
    boolean cubeControllerLeftStick; // If false use linear transformation from joystick input to power
                            // If true use cube transformation
                            // This concept was taken from nu23 of team 125 code.
    boolean cubeControllerRightStick;

    // The next three variables will contain cube values of deadbands used for cube driving.
    // This is done for performance reasons to reduce periodic calculations for cube driving.          
    private double cubeDeadbandX;
    private double cubeDeadbandY;
    private double cubeDeadbandOmega;
    
    public Controller(Constants.OIContants.ControllerDevice cd) {
        super(cd.getPortNumber()); // This needs to be done because the joystick parent class has a non-default constructor
        this.cd = cd;
        this.cdt = cd.getControllerDeviceType();  // This is done so the controller type is not 
                                                  // re-evaluated every time you need to get axis value
        
        /**
         * The deadband (d) indicates range of controller values that should be ignored. It must be positive when supplied here,
         * and the enforced deadband ignores the values between -d..+d, supplying 0 instead. The deadband is usually caused by the
         * controller springs being old and controller having a "slack" - returning to a value other than 0 when being released.
         * The easiest way to measure the deadband is in the driver station as it will display real-time controller values for
         * all analog axis. Note that "exercising" old joystics may greatly reduce the deadbands on specific axis. So do not give up
         * on the equipment that may still be used.
         */
        this.dx= cd.getDeadbandX();
        this.dy= cd.getDeadbandY();
        this.dm= cd.getDeadbandOmega();

        /**
         * These flags determie whether a particular joystic should use cube driving, which will use not a linear input (e.g. a value from joystick axis)
         * but a cube of that value. Since the joystic axis generates values from -1..1, the cube of that value in that range also will be between -1..1,
         * but it will provide much greater precision for the low speeds (in other words, will make speed adjustments in low speed range easier to make)
         * at the expense of the high speed range. Since the low speeds are often used when the robot needs to move very short distances, such change will
         * make small teleop position changes easier to do. The algorithm that we "borrowed" from team 125 nu23 code properly accounts for the
         * deadband range as well. We optimized it by reducing the need for recurring calculations, and made it a bit more readable by using ternary operators.
         */
        this.cubeControllerLeftStick=cd.isCubeControllerLeftStick();
        this.cubeControllerRightStick=cd.isCubeControllerRightStick();

        this.cubeDeadbandX = dx*dx*dx;
        this.cubeDeadbandY = dy*dy*dy;
        this.cubeDeadbandOmega = dm*dm*dm;
    } 

    /**
     * Controllers with a single joystick will only return left joystick values.
     */

    public double getLeftStickY() {
        double rawY;
        double result;

        switch(cdt){
            case LOGITECH:
                rawY = this.getY();
                break;
            case XBOX:
                rawY = this.getRawAxis(1);
                break;
            default:
                return 0; // Unknown controller type
        }
        if (this.cubeControllerLeftStick) {
            double cubeY = rawY*rawY*rawY;
            result = (cubeY - (rawY > 0 ? 1 : -1) * cubeDeadbandY)/(1 - cubeDeadbandY); // cubeController
            result = Math.abs(result) > this.cubeDeadbandY ? result : 0; // Ignores range of deadband values
        } else {
            result = (MathUtil.applyDeadband(rawY, dy)); // linear controller values
        }
        return result;
        
    }

    public double getLeftStickX() {
        double rawX;
        double result;

        switch(cdt){
            case LOGITECH:
                rawX = this.getX();
                break;
            case XBOX:
                rawX = this.getRawAxis(0);
                //System.out.println("rawX: " + rawX);
                break;
            default:
                return 0; // Unknown controller type
               
        }
        if (this.cubeControllerLeftStick) {
            double cubeX = rawX*rawX*rawX;
            result = (cubeX - (rawX > 0 ? 1 : -1) * cubeDeadbandX)/(1 - cubeDeadbandX); // cubeController
            result = Math.abs(result) > this.cubeDeadbandX ? result : 0; // Ignores range of deadband values
        } else {
            result = (MathUtil.applyDeadband(rawX, dx)); // linear controller values
        }
        return result;
        
    }

    public double getLeftStickOmega() {
        double rawOmega;
        double result;

        switch(cdt){
            case LOGITECH:
                rawOmega = this.getTwist();
                break;
            case XBOX:
                rawOmega = this.getRawAxis(3)-this.getRawAxis(2);
                break;
            default:
                return 0; // Unknown controller type
        }
        if (this.cubeControllerRightStick) {
            double cubeOmega = rawOmega*rawOmega*rawOmega;
            result = (cubeOmega - (rawOmega > 0 ? 1 : -1) * cubeDeadbandOmega)/(1 - cubeDeadbandOmega); // cubeController
            result = Math.abs(result) > this.cubeDeadbandOmega ? result : 0; // Ignores range of deadband values
        } else {
            result = (MathUtil.applyDeadband(rawOmega, dm)); // linear controller values
        }
        return result;

    }

    public double getRightStickY() {
        double rawY;
        double result;

        switch (cdt) {
            case XBOX:
                rawY = this.getRawAxis(5);
                break;
            default:
                return 0; // Unknown controller type
        }
        if (this.cubeControllerLeftStick) {
            double cubeY = rawY * rawY * rawY;
            result = (cubeY - (rawY > 0 ? 1 : -1) * cubeDeadbandY) / (1 - cubeDeadbandY); // cubeController
            result = Math.abs(result) > this.cubeDeadbandY ? result : 0; // Ignores range of deadband values
        } else {
            result = (MathUtil.applyDeadband(rawY, dy)); // linear controller values
        }
        return result;

    }

    public double getRightStickX() {
        double rawX;
        double result;

        switch (cdt) {
            case LOGITECH:
                rawX = this.getX();
                break;
            case XBOX:
                rawX = this.getRawAxis(4);
                // System.out.println("rawX: " + rawX);
                break;
            default:
                return 0; // Unknown controller type

        }
        if (this.cubeControllerLeftStick) {
            double cubeX = rawX * rawX * rawX;
            result = (cubeX - (rawX > 0 ? 1 : -1) * cubeDeadbandX) / (1 - cubeDeadbandX); // cubeController
            result = Math.abs(result) > this.cubeDeadbandX ? result : 0; // Ignores range of deadband values
        } else {
            result = (MathUtil.applyDeadband(rawX, dx)); // linear controller values
        }
        return result;

    }
}