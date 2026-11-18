public class SimulationResult {
 public double closestDistance; // distance to hub at closest approach
 public State bestState;        // ball state at closest approach
 public double time;            // time to reach that state

 public SimulationResult(double closestDistance, State bestState, double time) {
     this.closestDistance = closestDistance;
     this.bestState = bestState;
     this.time = time;
 }

 @Override
 public String toString() {
     return String.format("SimulationResult(closestDistance=%.3f, bestState=%s, time=%.3f)", 
                           closestDistance, bestState, time);
 }
}
