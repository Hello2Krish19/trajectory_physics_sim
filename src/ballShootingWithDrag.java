public class ballShootingWithDrag {

 static final double g = 9.81;
 static final double rho = 1.225; // air density kg/m^3
 static final double R = 0.0762; // ball radius (m) 
 static final double A = Math.PI * R * R;
 static final double Cd = 0.47;   // sphere drag coeff
 static final double Cl = 0.2;   // Magnus/lift coeff. Should be a Final but I had to change it for testing
 static final double mass = 0.145; // mass (kg) (baseball ~0.145 kg)
 static final double EPS = 1e-8;

 // spin vector (rad/s) in world frame
 Vector3 omega = new Vector3(0, 10.0, 0); 

 
 static Vector3 dragForce(Vector3 v_rel) {
     double speed = v_rel.magnitude();
     if (speed < 1e-9) return new Vector3(0,0,0);
     // Fd = -0.5 * rho * Cd * A * |v_rel| * v_rel
     double k = 0.5 * rho * Cd * A * speed;
     return v_rel.scale(-k);
 }

 static Vector3 liftForce(Vector3 v_rel, Vector3 omega) {
     double speed = v_rel.magnitude();
     if (speed < 1e-9) return new Vector3(0,0,0);
     Vector3 cross = omega.cross(v_rel);
     double crossMag = cross.magnitude();
     if (crossMag < 1e-9) return new Vector3(0,0,0);
     // direction = (omega x v_rel) normalized
     Vector3 dir = cross.scale(1.0 / crossMag);
     // magnitude = 0.5 * rho * Cl * A * speed^2
     double mag = 0.5 * rho * Cl * A * speed * speed;
     return dir.scale(mag);
 }

 // acceleration including gravity
 Vector3 acceleration(Vector3 v_world, Vector3 v_robot, Vector3 omega) {
	 
     Vector3 v_rel = v_world; 
     Vector3 Fd = dragForce(v_rel);
     Vector3 Fl = liftForce(v_rel, omega);
     Vector3 Fg = new Vector3(0, 0, -mass * g);
     Vector3 total = Fd.add(Fl).add(Fg);
     return total.scale(1.0 / mass);
 }

 void rk4Step(State s, double dt, Vector3 v_robot) {
     // State s has position p and velocity v (both world-frame)
     Vector3 p0 = s.p;
     Vector3 v0 = s.v;

     // k1
     Vector3 a1 = acceleration(v0, v_robot, omega);
     Vector3 k1v = a1.scale(dt);
     Vector3 k1p = v0.scale(dt);

     // k2
     Vector3 v2 = v0.add(k1v.scale(0.5));
     Vector3 a2 = acceleration(v2, v_robot, omega);
     Vector3 k2v = a2.scale(dt);
     Vector3 k2p = v2.scale(dt);

     // k3
     Vector3 v3 = v0.add(k2v.scale(0.5));
     Vector3 a3 = acceleration(v3, v_robot, omega);
     Vector3 k3v = a3.scale(dt);
     Vector3 k3p = v3.scale(dt);

     // k4
     Vector3 v4 = v0.add(k3v);
     Vector3 a4 = acceleration(v4, v_robot, omega);
     Vector3 k4v = a4.scale(dt);
     Vector3 k4p = v4.scale(dt);

     // update
     s.p = p0.add(k1p.add(k2p.scale(2.0)).add(k3p.scale(2.0)).add(k4p).scale(1.0/6.0));
     s.v = v0.add(k1v.add(k2v.scale(2.0)).add(k3v.scale(2.0)).add(k4v).scale(1.0/6.0));
 }

 // simulate trajectory until it passes hub Z or hits ground or max time
 // returns closest approach to hub (distance) and also final state when crossing hub plane
 SimulationResult simulate(Vector3 initialPos, Vector3 initialVelWorld, Vector3 hubPos, Vector3 v_robot) {
     double dt = 0.005; // tune for accuracy/perf
     State s = new State(initialPos.clone(), initialVelWorld.clone());
     double t = 0;
     double maxT = 5.0; // safety
     double bestDist = Double.POSITIVE_INFINITY;
     State bestState = null;
     
     double prevZ = initialPos.z;
     

     while (t < maxT) {
         // update best distance
         double dist = s.p.subtract(hubPos).magnitude();
         if (dist < bestDist) { 
        	 	bestDist = dist; bestState = new State(s.p.clone(), s.v.clone()); 
        	 }
         // if ball passed hub Z going downward and horizontal beyond hub plane, optional rules
         // We'll stop if the ball is below hubZ - 0.5 and descending
         if (s.p.z < -1.0) break; // hit floor
         // stop if ball has traveled over 100 meters
         if (Math.hypot(s.p.x - initialPos.x, s.p.y - initialPos.y) > 100.0) break;

         // integrate one step
         rk4Step(s, dt, v_robot);
         t += dt;

         if ((s.p.z - hubPos.z) < 0 && (prevZ - hubPos.z) > 0) {
            //crossed hub. If this triggers on the descending, we should break since we've already gone under the hub.
        	 	break;
         }
         prevZ = s.p.z;  // update previous z
     }
     // return best distance and state when closest
     return new SimulationResult(bestDist, bestState, t);
 }


 ShotSolution solveShot(Vector3 shooterPos, Vector3 hubPos, Vector3 v_robot, int angleSamples) {
     double heading = Math.atan2(hubPos.y - shooterPos.y, hubPos.x - shooterPos.x);

     ShotSolution best = null;
     for (int i = 0; i < angleSamples; ++i) {
         double frac = (i + 0.5) / angleSamples;
         // sample angle range 5 to 75 degrees (in radians)
         double thetaMin = Math.toRadians(5.0), thetaMax = Math.toRadians(75.0);
         double theta = thetaMin + (thetaMax - thetaMin) * frac;

         // now do binary search for speed (vLow..vHigh). Start with vLow small, vHigh big.
         double vLow = 1.0, vHigh = 50.0; // adjust ranges to your shooter. I have it from 1 m/s to 50 m/s
         for (int iter = 0; iter < 40; ++iter) {
             double vMid = 0.5 * (vLow + vHigh);
             // construct initial world-frame velocity for given heading+theta
             Vector3 vRobot = new Vector3(vMid * Math.cos(theta) * Math.cos(heading),
            		 						 vMid * Math.cos(theta) * Math.sin(heading),
            		 						 vMid * Math.sin(theta));
             Vector3 vel = vRobot.add(v_robot); //my naming conventions are perfect. this is the velocity of the ball relative to the world
             //System.out.println("vel magnitude: " + vel.magnitude() +" v_robot magnitude: " + v_robot.magnitude() + " vRobot.magintude: " + vRobot.magnitude());
             //^ also im not sure if I was supposed to add or subtract there
             SimulationResult res = simulate(shooterPos, vel, hubPos, v_robot);
             
            // System.out.printf("theta=%.2f deg, v=%.2f m/s, closestDist=%.3f m\n", Math.toDegrees(theta), vMid, res.closestDistance);
             
             double err = res.closestDistance;
             double dTarget = Math.hypot(hubPos.x - shooterPos.x, hubPos.y - shooterPos.y);
             double dAtClosest = Math.hypot(res.bestState.p.x - shooterPos.x, res.bestState.p.y - shooterPos.y);
             if (Math.abs(err) < 0.20) { // tolerance 20 cm feels fair. Idk how big the hub is I forgor

                 double wheelSpeedNeeded = vRobot.magnitude(); 
                 //what speed does the flywheel need to be to shoot the ball at the correct vRobot to reach the correct vel? 
                 ShotSolution sol = new ShotSolution(theta, vel.magnitude(), heading, wheelSpeedNeeded, err);
                 if (best == null || sol.err < best.err) best = sol;
                 break; // we did it once
             }
             // reset bounds of the binary search
             //if the shot falls short (peak horizontal distance < hub horizontal distance), increase speed
             if (dAtClosest < dTarget) {
                 vLow = vMid;
             } else {
                 vHigh = vMid;
             }
         } 
     } 

     return best;
 }
}


