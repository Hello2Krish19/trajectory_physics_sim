//READDDDDDDDDDDDDDDDDDDDDD MEEEEEEEEEEEEE

//ALL OF THOSE NUMBERS DOWN THERE ARE WRONGG!!!! WRONGGG!! I just needed random but somewhat reasonable numbers to make sure I didn't cause egregious issues

public class Main {

	public static void main(String[] args) {
        // --- Initialize solver ---
        ballShootingWithDrag solver = new ballShootingWithDrag();

        // --- Shooter and Hub positions (meters) ---
        // Example: robot at (0,0,0.8 m), hub at (5 m ahead, 1.2 m height)
        Vector3 shooterPos = new Vector3(0, 0, 0.8);
        Vector3 hubPos     = new Vector3(5, 2, 1.2);

        // --- Robot horizontal velocity (m/s) ---
        // Example: robot moving forward 10 m/s
        Vector3 robotVel = new Vector3(0.0, 0.0, 0.0);

        // --- Solve for shot ---
        int angleSamples = 20; // number of angles to sample in search
        ShotSolution solution = solver.solveShot(shooterPos, hubPos, robotVel, angleSamples);

        if (solution == null) {
            System.out.println("No valid shot found! Try increasing speed limits or changing spin.");
            return;
        }

        // --- Print solution ---
        System.out.printf("Best shot solution:\n");
        System.out.printf("  Elevation angle (theta) = %.2f degrees\n", Math.toDegrees(solution.theta));
        System.out.printf("  Heading (yaw)           = %.2f degrees\n", Math.toDegrees(solution.heading));
        System.out.printf("  World-frame speed       = %.2f m/s\n", solution.velocity);
        System.out.printf("  Required wheel speed    = %.2f m/s (after subtracting robot velocity)\n", solution.wheelSpeed);
        System.out.printf("  Closest distance error  = %.3f m\n", solution.err);

//        // --- Optional: simulate trajectory over time and print ---
//        System.out.println("\nSimulating trajectory...");
//        Vector3 initialVelWorld = new Vector3(
//                solution.v * Math.cos(solution.theta) * Math.cos(solution.heading),
//                solution.v * Math.cos(solution.theta) * Math.sin(solution.heading),
//                solution.v * Math.sin(solution.theta)
//        );
//
//        SimulationResult sim = solver.simulate(shooterPos, initialVelWorld, hubPos, robotVel);
//        double dt = 0.05; // print every 0.05 s
//        Vector3 pos = shooterPos.clone();
//        Vector3 vel = initialVelWorld.clone();
//        double t = 0;
//
//        while (pos.z > 0 && t < 5.0) {
//            System.out.printf("t = %.2f s: pos = (%.2f, %.2f, %.2f) m\n", t, pos.x, pos.y, pos.z);
//            solver.rk4Step(new State(pos, vel), dt, robotVel);
//            t += dt;
//        }

        System.out.println("Trajectory simulation complete.");
    }

}
