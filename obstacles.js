
//////////////////////////////////////////////
// Base (abstract) class for your obstacles //
//////////////////////////////////////////////
class Obstacle {
    constructor() {
      this.signMultiplier = 1; //outside
      this.color = color("white");
    }
  
    // Negates SDF to flip inside/outside.
    invert() {
      this.signMultiplier *= -1;
    }
  
    // OVERRIDE to draw object.
    draw() {}
  
    // Called when the ball collides with this obstacle. Override to make it do something useful, e.g., scoring, sound, colors, etc.
    notifyOfCollision() {}
  
    /**
     * OVERRIDE to implement signed distance at the specified point.
     * @param {2-array} p Position
     * @return {number} Returns the SDF value, d.
     */
    distance(p) {
      return 0 * this.signMultiplier;
    }
  
    /**
     * Convenience method that returns the SDF distance and object reference as an array.
     * @return {[number,object]} Returns the SDF value, d, and this object, o, as an array [d,o].
     */
    distanceO(p) {
      let result = [this.distance(p), this]; // MUST OVERRIDE TO RETURN PROPER distance()
      return [this.distance(p), this]; // MUST OVERRIDE TO RETURN PROPER distance()
    }
  
    // Normal at p
    normal(p) {
      return this.normalFD2(p);
    }
  
    normalFD2(p) {
      // TODO: student code, replace the following
      // re-implement FD2 from P1 using math.js
      return [1.0, 0];    
    }
  
    // False if the obstacle is not moving, e.g., fixed.
    isMoving() {
      return false;
    }
  
    getColor() {
      return this.color;
    }
    setColor(c) {
      this.color = c;
    }
  }
  
  ///////////////////////////////////////
  // Floor OBSTACLE
  ///////////////////////////////////////
  class FloorObstacle extends Obstacle {
    constructor(h) {
      super();
      this.h = h;
    }
  
    draw() {
      push();
      stroke(0, 0, 0);
      strokeWeight(0.1);
      line(-50, this.h, 50, this.h);
      pop();
    }
  
    distance(p) {
      let result = this.signMultiplier * (this.h - p[1]);
      return result;
    }
  }
  
  ////////////////////////////////////////////////////////////
  // Some 2D Signed Distance Fields. Feel free to add more! //
  // See https://iquilezles.org/articles/distfunctions2d    //
  ////////////////////////////////////////////////////////////
  class SDF {
    /**
     * Computes SDF union distance value, min(d1,d2), and its corresponding object reference.
     * @param distO1 {[number,object]} Obstacle #1's SDF value, d, and object ref, o, as an array [d1,o1].
     * @param distO2 {[number,object]} Obstacle #2's SDF value, d, and object ref, o, as an array [d2,o2].
     * @return {[number,object]} Returns [d1,o1] if d1<d2, and [d2,o2] otherwise.
     */
    static opUnion(distO1, distO2) {
      return distO1[0] < distO2[0] ? distO1 : distO2;
    }
    /**
     * Computes SDF intersection distance value, max(d1,d2), and its corresponding object reference.
     * @param distO1 {[number,object]} Obstacle #1's SDF value, d, and object ref, o, as an array [d1,o1].
     * @param distO2 {[number,object]} Obstacle #2's SDF value, d, and object ref, o, as an array [d2,o2].
     * @return {[number,object]} Returns [d1,o1] if d1>d2, and [d2,o2] otherwise.
     */
    static opIntersection(distO1, distO2) {
      return distO1[0] > distO2[0] ? distO1 : distO2;
    }
  
    // ANOTHER BOOLEAN YOU MAY WANT TO USE:
    //float opSubtraction ( float d1, float d2 ) { return max(-d1,d2);}
  
    /**
     * SDF of a circle at origin.
     * @param {2-array}   p Evaluation point
     * @param {number}    r Radius
     * @return {number}   SDF value
     */
    static sdCircle(p, r) {
      let result = math.norm(p) - r;
      return result;
    }
  }