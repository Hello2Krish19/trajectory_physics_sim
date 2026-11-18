public class State {
 public Vector3 p;  // position
 public Vector3 v;  // velocity

 public State(Vector3 p, Vector3 v) {
     this.p = p;
     this.v = v;
 }

 public State clone() {
     return new State(p.clone(), v.clone());
 }

 @Override
 public String toString() {
     return String.format("State(position=%s, velocity=%s)", p, v);
 }
}	