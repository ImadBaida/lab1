import simlejos.ExecutionController;
import simlejos.hardware.motor.Motor;
import simlejos.hardware.port.SensorPort;
import simlejos.hardware.sensor.EV3UltrasonicSensor;
import simlejos.robotics.RegulatedMotor;

/**
 * Main class of the program.
 */
public class Lab1 {
  
  /** The maximum distance detected by the ultrasonic sensor, in cm. */
  public static final int MAX_SENSOR_DIST = 255;
  
  //Parameters: adjust these for desired performance. You can also add new ones.
  
  /** Ideal distance between the sensor and the wall (cm). */
  public static final int WALL_DIST = 30;
  /** The maximum tolerated deviation from the ideal WALL_DIST, aka the deadband, in cm. */
  public static final int WALL_DIST_ERR_THRESH = 5;
  /** Speed of the slower wheel (deg/sec). */
  public static final int MOTOR_LOW = 30;
  /** Speed of the faster wheel (deg/sec). */
  public static final int MOTOR_HIGH = 200;
  /** The limit of invalid samples that we read from the US sensor before assuming no obstacle. */
  public static final int INVALID_SAMPLE_LIMIT = 20;

  // Hardware resources

  /** The ultrasonic sensor. */
  public static final EV3UltrasonicSensor usSensor = new EV3UltrasonicSensor(SensorPort.S1);
  /** The left motor. */
  public static final RegulatedMotor leftMotor = Motor.A;
  /** The right motor. */
  public static final RegulatedMotor rightMotor = Motor.D;
  
  // Other class variables
  
  /** The distance remembered by the filter() method. */
  private static int prevDistance;
  /** The number of invalid samples seen by filter() so far. */
  private static int invalidSampleCount;

  // These arrays are class variables to avoid creating new ones at each iteration.
  /** Buffer (array) to store US samples. */
  private static float[] usData = new float[usSensor.sampleSize()];
  /** The left and right motor speeds, respectively. */
  private static int[] motorSpeeds = new int[2];
  
  private static final int LEFT = 0;
  private static final int RIGHT = 1;


  /** Main entry point. */
  public static void main(String[] args) {
    System.out.println("Starting Lab 1 demo");

    ExecutionController.performPhysicsStepsInBackground();
    
    leftMotor.setSpeed(MOTOR_HIGH);
    rightMotor.setSpeed(MOTOR_HIGH);
    goForward();
    
    while (true) {
      controller(readUsDistance(), motorSpeeds);
      setMotorSpeeds();
    }
  }
  
  /**
   * Process a movement based on the US distance passed in (eg, Bang-Bang or Proportional style).
   *
   * @param distance the distance to the wall in cm
   * @param motorSpeeds output parameter you need to set
   */
  public static void controller(int distance, int[] motorSpeeds) {
    int leftSpeed;
    int rightSpeed;
    //error calculated by subtracting actual distance from demanded distance 
    int error = WALL_DIST - distance; 
    //Triggers if not within deadband
    if (Math.abs(error) > WALL_DIST_ERR_THRESH) {
      if (error > 0) {
        //Distance is lower than desired; speed up left wheel to turn away from the wall
        leftSpeed = MOTOR_HIGH;
        rightSpeed = MOTOR_LOW;
        goForward();
      } else {
        //Distance is higher than desired; speed up right wheel to turn towards the wall
        leftSpeed = MOTOR_LOW;
        rightSpeed = MOTOR_HIGH;
        goForward();
      }
    } else {
      leftSpeed = MOTOR_HIGH;
      rightSpeed = MOTOR_HIGH; // if distance is within the deadband, return to normal speed
      goForward();
    }
    
    motorSpeeds[LEFT] = leftSpeed;
    motorSpeeds[RIGHT] = rightSpeed;
  }
  
  /** Tells the motors to go forward. */
  
  public static void goForward() {
    leftMotor.backward();
    rightMotor.backward();
  }
  
  /** Returns the filtered distance between the US sensor and an obstacle in cm. */
  public static int readUsDistance() {
    usSensor.fetchSample(usData, 0);
    // extract from buffer, convert to cm, cast to int, and filter
    return filter((int) (usData[0] * 100.0));
  }
  
  /**
   * Rudimentary filter - toss out invalid samples corresponding to null signal.
   *
   * @param distance raw distance measured by the sensor in cm
   * @return the filtered distance in cm
   */
  static int filter(int distance) {
    if (distance >= MAX_SENSOR_DIST && invalidSampleCount < INVALID_SAMPLE_LIMIT) {
      // bad value, increment the filter value and return the distance remembered from before
      invalidSampleCount++;
      return prevDistance;
    } else {
      if (distance < MAX_SENSOR_DIST) {
        // distance went below MAX_SENSOR_DIST: reset filter and remember the input distance.
        invalidSampleCount = 0;
      }
      prevDistance = distance;
      return distance;
    }
  }
  
  /** Sets the speeds of the left and right motors from the motorSpeeds array. */
  public static void setMotorSpeeds() {
    leftMotor.setSpeed(motorSpeeds[LEFT]);
    rightMotor.setSpeed(motorSpeeds[RIGHT]);
    //force update
  }

}
