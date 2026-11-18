//READDDDDDDDDDDDDDDDDDDD MEEEEEEEEEEEEE

//ALL OF THOSE NUMBERS DOWN THERE ARE WRONGG!!!! WRONGGG!! I just needed random but somewhat reasonable numbers to make sure I didn't cause egregious issues

import java.util.Scanner;

public class Main {

	public static void main(String[] args) {
        // --- Initialize solver ---
        ballShootingWithDrag solver = new ballShootingWithDrag();

        Scanner sc = new Scanner(System.in);

        // --- Shooter and Hub positions (meters) ---
        // args: tx ty tz [sx sy sz [vx vy vz]]   otherwise type them in
        Vector3 hubPos;
        Vector3 shooterPos;
        Vector3 robotVel;
        if (args.length >= 3) {
            hubPos     = new Vector3(Double.parseDouble(args[0]), Double.parseDouble(args[1]), Double.parseDouble(args[2]));
            shooterPos = args.length >= 6
                    ? new Vector3(Double.parseDouble(args[3]), Double.parseDouble(args[4]), Double.parseDouble(args[5]))
                    : new Vector3(0, 0, 1);
            robotVel   = args.length >= 9
                    ? new Vector3(Double.parseDouble(args[6]), Double.parseDouble(args[7]), Double.parseDouble(args[8]))
                    : new Vector3(0.0, 0.0, 0.0);
        } else {
            System.out.print("Target position x y z (m): ");
            hubPos = new Vector3(sc.nextDouble(), sc.nextDouble(), sc.nextDouble());
            System.out.print("Shooter position x y z (m)  [type 0 0 1 if default]: ");
            shooterPos = new Vector3(sc.nextDouble(), sc.nextDouble(), sc.nextDouble());
            System.out.print("Robot velocity vx vy vz (m/s)  [type 0 0 0 if stationary]: ");
            robotVel = new Vector3(sc.nextDouble(), sc.nextDouble(), sc.nextDouble());
        }

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
