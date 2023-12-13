
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
      return [this.distance(p), this]; // MUST OVERRIDE TO RETURN PROPER distance() 
      // let result = [this.distance(p), this]; // MUST OVERRIDE TO RETURN PROPER distance()
      // return [this.distance(p), this]; // MUST OVERRIDE TO RETURN PROPER distance()
    }
  
    // Normal at p
    normal(p) {
      return this.normalFD2(p);
    }
  
    normalFD2(p) {
      // TODO: student code, replace the following
      // re-implement FD2 from P1 using math.js
      const h = 0.1;
      let q = createVector(p.x, p.y);
      q.x += h;
      let d = this.distance(p);
      let dxh = this.distance(q);
      q.x -= h;
      q.y += h;
      let dyh = this.distance(q);
  
      //replace d with r
      let r = createVector(p.x, p.y);
      r.x -= h;
      let r_dxh = this.distance(r);
      r.x += h;
      r.y -= h;
      let r_dyh = this.distance(r);
  
      let n = createVector((dxh - r_dxh) / (2*h), (dyh - r_dyh) / (2*h)).normalize();
      return n;
  

      // const h = 0.1;
      // let q = [p[0], p[1]];
      // q[0] += h;
      // let d = this.distance(p);
      // let dxh = this.distance(q);
      // q[0] -= h;
      // q[1] += h;
      // let dyh = this.distance(q);
  
      // //replace d with r
      // let r = [p[0], p[1]];
      // r[0] -= h;
      // let r_dxh = this.distance(r);
      // r[0] += h;
      // r[1] -= h;
      // let r_dyh = this.distance(r);
  
      // let n = math.norm([(dxh - r_dxh) / (2*h), (dyh - r_dyh) / (2*h)]);
      // return n;
  
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
// Axis-Aligned BOX OBSTACLE
///////////////////////////////////////
class BoxObstacle extends Obstacle {

	/** Rect in RADIUS mode
	 * @param {p5.Vector} center Center of box.
	 * @param {number} bx Half-width in X
	 * @param {number} by Half-width in Y
	 */
	constructor(center, bx, by) {
		super();
		this.center = center;
		this.bx = bx;
		this.by = by;
	}

	draw() {
		push();
		rectMode(RADIUS);
		if (this.signMultiplier > 0) {
      strokeWeight(0);
			fill(this.color);
			rect(this.center.x, this.center.y, this.bx, this.by); // RADIUS MODE
		}
		pop();
	}

	distance(p) {
		let v = p5.Vector.sub(p, this.center); // p-c
		return this.signMultiplier * SDF.sdBox(v, vec2(this.bx, this.by));
	}
}

  ///////////////////////////////////////
// CIRCLE OBSTACLE
///////////////////////////////////////
class CircleObstacle extends Obstacle {
	constructor(center, radius) {
		super();
		this.center = center;
		this.radius = radius;
		this.hit = false;
		this.text = "";
		this.time = deltaTime;
	}

	draw() {
		// if (this.signMultiplier > 0) {
      push();
      strokeWeight(0);
			fill(this.color);
			circle(this.center.x, this.center.y, this.radius*1.3); // RADIUS MODE
      pop();
		// }
	}
  notifyOfCollision() {
		// score = score + 10;
		// let beige = color(244, 221, 181);
		// this.text = "+10";
		// if (this.hit) {
		// 	this.color = beige;
		// }
		// else {
		// 	this.color = color(210, 76, 58);
		// }
		this.hit=!this.hit;
	}
	distance(p) {
		return this.signMultiplier * SDF.sdCircle(sub(p, this.center), this.radius);
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
      strokeWeight(.5);
      line(-50, this.h*1.1, 50, this.h*1.1);
      pop();
    }
  
    distance(p) {
      let result = this.signMultiplier * (this.h - p.y);
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
      return (distO1[0] < distO2[0]) ? distO1 : distO2;
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
    // static sdCircle(p, r) {
    //   let result = math.norm(p) - r;
    //   return result;
    // }
    static sdCircle(p, r) {
      return length(p) - r;
    }
    	/**
     * SDF of a box at origin
     * @param {p5.Vector} p Evaluation point
     * @param {p5.Vector} b Half widths in X & Y
     * @return {number}   SDF value
     */
    static sdBox(p, b) {
      let d = sub(absv2(p), b);
      return length(maxv2(d, 0.0)) + min(max(d.x, d.y), 0.0);
    } 
  }
  
/////////////////////////////////////////////////////////////////
// Some convenient GLSL-like macros for p5.Vector calculations //
/////////////////////////////////////////////////////////////////
function length(v) {
	return v.mag();
}

function dot(x, y) {
	return x.dot(y);
}

function dot2(x) {
	return x.dot(x);
}

function vec2(a, b) {
	return createVector(a, b);
}

function vec3(a, b, c) {
	return createVector(a, b, c);
}

function sign(n) {
	return Math.sign(n);
}

function clamp(n, low, high) {
	return constrain(n, low, high);
}

function add(v, w) {
	return p5.Vector.add(v, w);
}

function sub(v, w) {
	return p5.Vector.sub(v, w);
}

function absv2(v) {
	return vec2(Math.abs(v.x), Math.abs(v.y));
}

function maxv2(v, n) {
	return vec2(Math.max(v.x, n), Math.max(v.y, n));
}

function minv2(v, n) {
	return vec2(Math.min(v.x, n), Math.min(v.y, n));
}

function vertexv2(p) {
	vertex(p.x, p.y);
}

function mult(w, a) {
	let v = createVector(0,0);
	v.x = a * w.x;
	v.y = a * w.y;
	return v;
}
 
function acc(v, a, w) {
	v.x += a * w.x;
	v.y += a * w.y;
}

function rotateVec2(v, thetaRad) {
	const c = cos(thetaRad);
	const s = sin(thetaRad);
	return vec2(c * v.x - s * v.y, s * v.x + c * v.y);
}