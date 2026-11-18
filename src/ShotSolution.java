public class ShotSolution {
 public double theta;      // launch elevation angle in radians
 public double velocity;   // initial speed magnitude (m/s)
 public double heading;    // yaw/heading angle in radians
 public double wheelSpeed; // speed relative to robot (for shooter wheel)
 public double err;        // error to target (distance to hub)

 public ShotSolution(double theta, double velocity, double heading, double wheelSpeed, double err) {
     this.theta = theta;
     this.velocity = velocity;
     this.heading = heading;
     this.wheelSpeed = wheelSpeed;
     this.err = err;
 }

 @Override
 public String toString() {
     return String.format("ShotSolution(theta=%.3f deg, velocity=%.2f m/s, heading=%.3f deg, wheelSpeed=%.2f m/s, err=%.3f m)",
             Math.toDegrees(theta), velocity, Math.toDegrees(heading), wheelSpeed, err);
 }
}
