public class Vector3 {
 public double x, y, z;

 public Vector3(double x, double y, double z) {
     this.x = x;
     this.y = y;
     this.z = z;
 }

 public Vector3 clone() {
     return new Vector3(x, y, z);
 }

 public Vector3 add(Vector3 v) {
     return new Vector3(x + v.x, y + v.y, z + v.z);
 }

 public Vector3 subtract(Vector3 v) {
     return new Vector3(x - v.x, y - v.y, z - v.z);
 }

 public Vector3 scale(double s) {
     return new Vector3(x * s, y * s, z * s);
 }

 public double dot(Vector3 v) {
     return x * v.x + y * v.y + z * v.z;
 }

 public Vector3 cross(Vector3 v) {
     return new Vector3(
         y * v.z - z * v.y,
         z * v.x - x * v.z,
         x * v.y - y * v.x
     );
 }

 public double magnitude() {
     return Math.sqrt(x*x + y*y + z*z);
 }

 // (unit vector)
 public Vector3 normalize() {
     double mag = magnitude();
     if (mag < 1e-8) return new Vector3(0,0,0); // prevent divide by zero
     return scale(1.0 / mag);
 }

 @Override
 public String toString() {
     return String.format("Vector3(%.3f, %.3f, %.3f)", x, y, z);
 }
}