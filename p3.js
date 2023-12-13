// We'll use math.js for matrix operations in JavaScript
// Include math.js in your Processing project

const FRAMERATE = 30;
const floor_h = 2.0; // floor height, note that gravity is reversed (positive y is down)
let transform_list = []; // list of all the Transformation objects
let name_to_transform = {}; // map from name to Transformation object
let dof_list = []; // list of all the Transformation objects with non-zero degrees of freedom

let ees; // list of end effectors (also Transformation objects)
let targets = []; // list of 2-arrays, target positions of the end effectors
let q; // list of DoF values (translation and hinge), N-array, same length as dof_list since we only have 1-dof Transformations

// used only in physics mode
let CoM_xya; // x, y postion and rotation of the CoM, will be updated during simulation, 3-vector
let d_CoM_xya; // x, y, rotation velocities of the CoM, will be updated during simulation
let init_offset_com_2_base;
let total_mass; // total mass of the rods, constant, only updated once before each physics sim stage
let I; // moment of inertia around CoM, constant, only updated once before each physics sim stage
const density = 5.0; // density of the bone segments, kg/m
let last_timestep_positions;

let obstacles = []; // list of Obstacle objects, in starter code there is only one floor

let bone_segments = [];
let collision_candidates = null;

// visualization constants
const g_s = 150;
let x_offset_draw, y_offset_draw;

let physics_on = false;
let physics_timer = -1;
const physics_duration = 5; // seconds

let dragging_idx = -1;
let remaining_drags;

class Segment {
  constructor(start, end) {
    // start and end are 2d vectors, starting and ending points of the rod
    this.s = start;
    this.e = end;

    // constants
    this.length = math.norm(math.subtract(start, end));
    this.m = this.length * density;
    // https://en.wikipedia.org/wiki/List_of_moments_of_inertia
    this.I_com = (1 / 12.0) * this.m * this.length * this.length;
  }

  get_global_com_xy() {
    return math.multiply(0.5, math.add(this.s, this.e));
  }
  get_length() {
		return this.length;
	}
	get_mass() {
		return this.m;
	}
	get_inertia() {
		return this.I_com;
	}

  draw() {
    push();
    stroke(128, 128, 0);
    strokeWeight(0.1);
    line(this.s[0], this.s[1], this.e[0], this.e[1]);
    pop();
  }
}

class CollisionCandidates {
  // store the global-frame start and end points of all the rods
  // velocities of the points are also stored, through finite difference
  // for more convenient collision detection
  constructor(list_segments) {
    self.points = []; // a list of 2d vectors
    list_segments.forEach((seg) => {
      self.points.push(seg.s);
      self.points.push(seg.e);
    });

    self.points_v = self.points.map((x) => math.multiply(0.0, x)); // initialize, a list of 2d vectors
  }

  update_points(list_segments, dt) {
    let new_points = [];
    list_segments.forEach((seg) => {
      new_points.push(seg.s);
      new_points.push(seg.e);
    });

    self.points_v = self.points.map((x, i) =>
      math.multiply(math.subtract(new_points[i], x), 1.0 / dt)
    );
    self.points = new_points;
  }

  length() {
    return self.points.length;
  }

  get_p(i) {
    return self.points[i];
  }

  get_v(i) {
    return self.points_v[i];
  }

  draw() {
    push();
    self.points.forEach((p) => {
      noStroke();
      fill(0, 0, 0);
      circle(p[0], p[1], 0.1);
    });
    pop();
  }
}

function setup() {
  createCanvas(windowWidth, windowHeight);
  x_offset_draw = windowWidth * 0.5;
  y_offset_draw = windowHeight * 0.5;
  background(255);

  setup_character();

  ees = [
    name_to_transform["l_hand"],
    name_to_transform["r_hand"],
    name_to_transform["l_foot"],
    name_to_transform["r_foot"],
  ];
  remaining_drags = ees.length - 1;

  // Initialize joint positions
  let ndofs = dof_list.length;
  // q = math.random([ndofs]);
  q = math.zeros([ndofs]);
  q[1] = 0; // make sure initial y for all bones are above the floor
  set_joint_positions(q, 0);

  reset_targets();

  // TODO: ADD YOUR MANY WONDERFUL OBSTACLES/TERRAINS
  let lineK = new FloorObstacle(floor_h);
  obstacles.push(lineK);
  let circleK = new CircleObstacle(createVector(floor_h, floor_h*0.2), floor_h);
  circleK.setColor("red");
  // circleK.setStrokeColor(color(240, 106, 88));
  // circleK.setCOR(0.85);
  //circleK.setEnergy(500); //energy modeled as just adding the unit normal multiplied by some scalar
  obstacles.push(circleK);

  unit_test_J();
  IK_unit_test();
}

function reset_targets() {
  // reset targets to current end effector positions, meaning zero IK residual
  // targets being a list of 2-arrays
  targets = [];
  for (let i = 0; i < ees.length; i++) {
    let ee = ees[i];
    let pos = ee.global_position();
    targets.push(pos);
  }
}


function set_joint_positions(q, dt) {
  for (let i = 0; i < q.length; i++) {
    let q_i = q[i];
    let jnt_i = dof_list[i];
    jnt_i.set_dof(q_i);
    jnt_i.local_derivative();
  }
  // update the bone segments accordingly
  update_current_segments_and_collision(dt);
}

function update_current_segments_and_collision(dt) {
  bone_segments = [];
  

  transform_list.forEach((tr) => {
    // excluding all base_ transforms
    if (tr.name.indexOf("base_") < 0) {
      let xy = tr.global_position();
      let xy_parent = tr.parent.global_position();
      bone_segments.push(new Segment(xy_parent, xy));
    }
  });

  if (collision_candidates == null) {
    collision_candidates = new CollisionCandidates(bone_segments);
    last_timestep_positions = new CollisionCandidates(bone_segments);
  } else {
    collision_candidates.update_points(bone_segments, dt);
  }
}

// only call once before each physics stage
function get_init_CoM_xy(return_total_mass) {
  // get CoM of the rods from the segments
  // return_total_mass: boolean, whether to return total mass as well

  // TODO: STUDENT CODE BEGIN
  // replace this with your code

  // TODO: STUDENT CODE BEGIN
  // replace this with your code
  let CoM = [0.0, 0.0];
	let total_mass = 0.0;
	for (let bone of bone_segments) {
		let scalar = bone.get_length() * bone.get_mass();
		let midpoint = bone.get_global_com_xy();
		let product = math.multiply(scalar, midpoint);
		CoM = math.add(CoM, product);
		total_mass += bone.get_mass();
	}
	let total_mass_factor = 1.0 / total_mass;
	CoM = math.multiply(total_mass_factor, CoM);
	// Sum Length * mass * COM of each rod
	// Divide it by the total mass of the rods
  // STUDENT CODE END
  // STUDENT CODE END

  if (return_total_mass) {
    return [CoM, total_mass];
  } else {
    return CoM;
  }
}

// only call once before each physics stage
function get_moment_of_inertia_around_CoM(com) {
  // caculate total moment of inertia of the ridig body around CoM from the segments
  // com: the center of mass of the rigid body (2-vector)

  // TODO: STUDENT CODE BEGIN
  // replace this with your code

  let I_total = 0.0;
	// Get individual inertias
	// Parallel Axis Theorem around CoM
	// Add their inertias
	for (let bone of bone_segments) {
		let d = math.norm(math.subtract(bone.get_global_com_xy(), com));
		let d_square = d * d;
		let inertia = d_square + bone.get_inertia();
		I_total += inertia;
	}
  // STUDENT CODE END

  return I_total;
}

function cross_2d(a, b) {
  return a[0] * b[1] - a[1] * b[0];
}

// only call once before each physics stage
function init_phys_state() {
  // resolve init collisions from the end state of IK.
  let NUM_PERTURB_ITERS = 100;
  let l = collision_candidates.length();

  for (let j = 0; j < NUM_PERTURB_ITERS; j++) {
    let in_collision = false;
    for (let i = 0; i < l; i++) {
      let p = collision_candidates.get_p(i);
      collision_output = collisionCheck(p, obstacles);
      if (collision_output[0] > 0) {
        in_collision = true;
        base_correction = math.multiply(
          collision_output[0],
          collision_output[1]
        );
        q[0] += base_correction[0];
        q[1] += base_correction[1];
        set_joint_positions(q, 0);
      }
    }
    if (!in_collision) {
      print("finish pre-collision " + str(j));
      break;
    }
  }

  // init CoM state
  res = get_init_CoM_xy(true);
  CoM_xya = res[0];
  CoM_xya.push(0.0); // init rotation is a frame of reference we set
  total_mass = res[1];
  d_CoM_xya = [0.0, 0.0, 0.0];

  I = get_moment_of_inertia_around_CoM(CoM_xya.slice(0, 2));

  print("mass " + total_mass);
  print("I " + I);

  init_offset_com_2_base = math.subtract(q.slice(0, 2), CoM_xya.slice(0, 2));
  init_offset_com_2_base.push(q[2]);
}

function terminate_phys_state() {
  CoM_xya = undefined;
  d_CoM_xya = undefined;
  init_offset_com_2_base = undefined;
  I = undefined;
  total_mass = undefined;
}

function set_q_from_phys_state(dt) {
  // set base pos according to CoM pos
  // base velocities are ignored since it's not used in IK mode or visualization
  last_timestep_positions = new CollisionCandidates(bone_segments);
  let a = CoM_xya[2];
  let rot_mat = [
    [math.cos(a), -math.sin(a)],
    [math.sin(a), math.cos(a)],
  ];
  let offset_final = math.multiply(rot_mat, init_offset_com_2_base.slice(0, 2));
  q[0] = CoM_xya[0] + offset_final[0];
  q[1] = CoM_xya[1] + offset_final[1];
  q[2] = init_offset_com_2_base[2] + CoM_xya[2];
  set_joint_positions(q, dt);
}

/** SDF of all scene obstacles.
 * @param {p5.Vector} p 2D position to query SDF at.
 * @return {number,object} [d,o] Distance, d, to closest obstacle, o.
 */
function distanceO(p) {
	let minDistO = [Infinity, this];
  let pvec = createVector(p[0],p[1]);
	for (let i = 0; i < obstacles.length; i++) {
		const doi = obstacles[i].distanceO(pvec);
		minDistO = SDF.opUnion(minDistO, doi); // [min(d1,d2), closestObstacle]
	}
	return minDistO;
}

function take_physics_step(fx, fy, tau) {
  // fx, fy, tau are scalars, pass by value

  let substep = 10;
  let dt = 1.0 / (substep * FRAMERATE);

  for (let i = 0; i < substep; i++) {
    // TODO: STUDENT CODE BEGIN

    // run one step of rigid-body simulation
    // fx, fy, tau are external forces and torque (i.e. gravity) excluding collision
    // add your collision forces to fx, fy, tau
    // then integrate the equations of motion
		// let floor_y = windowHeight * 0.5 + (100.0*(h+0.1));
		for (let i = 0; i < collision_candidates.length(); i++){
			let p = collision_candidates.get_p(i);
			// Check if close to floor
      let dist = distanceO(p);
			if (dist[0] <= 0.1) {
        print("dist: "+dist)
        // if (p[1] >= (floor_h - 0.05)) {
				// Find penalty force
				let last_p = last_timestep_positions.get_p(i);
				fx += -0.01*(p[0]-last_p[0]);
				fy += -10.0*(p[1]-0.05);
				// Find torque
				let p_three = [p[0], p[1], 0.0];
				let com_xy = [CoM_xya[0], CoM_xya[1], 0.0];
				let fk = [fx, fy, 0.0];
				let moment_arm = math.subtract(com_xy, p_three);
				// Cross product force and moment arm to get torque
				let tau_k = math.norm(math.cross(fk, moment_arm));
				// Damping the torque
				tau += (tau_k * 0.1);
			}
		}
		
		let a = [fx/total_mass, fy/total_mass, tau/I];
		// update d_CoM_xya -- the velocity using the acceleration
		d_CoM_xya = math.add(d_CoM_xya, math.multiply(dt, a));
    // STUDENT CODE END

    CoM_xya = math.add(CoM_xya, math.multiply(dt, d_CoM_xya));

    set_q_from_phys_state(dt);
  }
}

function draw() {
  clear();
  translate(x_offset_draw, y_offset_draw);
  scale(g_s);
  background(255);

  text_x = (-x_offset_draw * 0.8) / g_s;
  text_y = (-y_offset_draw * 0.8) / g_s;

  if (physics_on) {
    take_physics_step(0.0, 0.5 * total_mass, 0.0); // (moon~~~) gravity

    physics_timer -= 1;
    push();
    fill(0, 0, 0);
    textSize(0.15);
    text("Physics " + str(physics_timer), text_x, text_y);
    pop();

    reset_targets();

    if (physics_timer <= 0) {
      physics_on = false;
      terminate_phys_state();
      remaining_drags = ees.length - 1;
    }
  } else {
    push();
    fill(0, 0, 0);
    textSize(0.15);
    text("Kinematics " + str(remaining_drags), text_x, text_y);
    pop();

    // let g_collision = math.zeros([q.length]);

    for (let i = 0; i < 4; i++) {
      let g = take_IK_step(ees, targets);
      q = math.add(q, math.multiply(-0.2, g)); // feel free to change this step size
      set_joint_positions(q, 1.0 / FRAMERATE);
    }
  }
  
  draw_character();
  draw_targets();

  if (dragging_idx >= 0) {
    let x = math.divide(mouseX - x_offset_draw, g_s);
    let y = math.divide(mouseY - y_offset_draw, g_s);
    push();
    stroke(128, 0, 128);
    fill(128, 0, 128);
    strokeWeight(0.05);
    circle(x, y, 0.2);
    pop();
  }

  // DRAW OBSTACLES:
  push();
  fill(255);
  for (let i = 0; i < obstacles.length; i++) obstacles[i].draw();
  pop();

  // noLoop();
}

function draw_targets() {
  for (i = 0; i < targets.length; i++) {
    push();
    strokeWeight(0.05);
    let target = targets[i];
    let size;
    if (i < 2) {
      stroke(0, 128, 0);
      size = 0.1;
    } else {
      stroke(0, 0, 128);
      size = 0.15;
    }
    if (dragging_idx == i) {
      stroke(128, 0, 0);
    }
    circle(target[0], target[1], size);

    pop();
  }
}

function print_math_js(math_obj, precision = 6) {
  let formatted_obj = math.format(math_obj, { precision: precision });
  print(formatted_obj);
}

function transform_between(start, end) {
  // start and end are Transformation objects
  // return the transformation matrix from start to end
  if (start == end) {
    return math.identity([3]);
  } else {
    return math.multiply(
      transform_between(start, end.parent),
      end.local_transform()
    );
  }
}



function compute_jacobian(end_effector) {
  // end_effector is a Transformation object
  // return the Jacobian matrix of the end_effector, a 2xN matrix
  // do this by computing the derivative of the end_effector's global position
  // with respect to the degrees of freedom, column by column

  let ndofs = dof_list.length;
  let J = math.zeros([2, ndofs]);
  //I THINK end_effector is the Transformation object of one of our end effectors. for example, the Object with name: r_foot
  //unpack all the parents
  let columnVector = [[0],[0],[1]];
  
  if (end_effector.name === "l_hand") {
    let d0 = math.multiply(transform_list[0].dT, transform_list[1].T, transform_list[2].T, transform_list[3]._value, transform_list[5]._value, transform_list[6].T, transform_list[7]._value, transform_list[8].T, transform_list[9]._value, columnVector);
    let d1 = math.multiply(transform_list[0].T, transform_list[1].dT, transform_list[2].T, transform_list[3]._value, transform_list[5]._value, transform_list[6].T, transform_list[7]._value, transform_list[8].T, transform_list[9]._value, columnVector);
    let d2 = math.multiply(transform_list[0].T, transform_list[1].T, transform_list[2].dT, transform_list[3]._value, transform_list[5]._value, transform_list[6].T, transform_list[7]._value, transform_list[8].T, transform_list[9]._value, columnVector);
    let d3 = math.multiply(transform_list[0].T, transform_list[1].T, transform_list[2].T, transform_list[3]._value, transform_list[5]._value, transform_list[6].dT, transform_list[7]._value, transform_list[8].T, transform_list[9]._value, columnVector);
    let d4 = math.multiply(transform_list[0].T, transform_list[1].T, transform_list[2].T, transform_list[3]._value, transform_list[5]._value, transform_list[6].T, transform_list[7]._value, transform_list[8].dT, transform_list[9]._value, columnVector);
    //print(d0);
    J[0][0] = d0[0][0];
    J[1][0] = d0[1][0];
    J[0][1] = d1[0][0];
    J[1][1] = d1[1][0];
    J[0][2] = d2[0][0];
    J[0][3] = d3[0][0];
    J[1][3] = d3[1][0];
    J[0][4] = d4[0][0];
    J[1][4] = d4[1][0];
  }
  else if (end_effector.name === "r_hand") {
    let d0 = math.multiply(transform_list[0].dT, transform_list[1].T, transform_list[2].T, transform_list[3]._value, transform_list[11]._value, transform_list[12].T, transform_list[13]._value, transform_list[14].T, transform_list[15]._value, columnVector);
    let d1 = math.multiply(transform_list[0].T, transform_list[1].dT, transform_list[2].T, transform_list[3]._value, transform_list[11]._value, transform_list[12].T, transform_list[13]._value, transform_list[14].T, transform_list[15]._value, columnVector);
    let d2 = math.multiply(transform_list[0].T, transform_list[1].T, transform_list[2].dT, transform_list[3]._value, transform_list[11]._value, transform_list[12].T, transform_list[13]._value, transform_list[14].T, transform_list[15]._value, columnVector);
    let d5 = math.multiply(transform_list[0].T, transform_list[1].T, transform_list[2].T, transform_list[3]._value, transform_list[11]._value, transform_list[12].dT, transform_list[13]._value, transform_list[14].T, transform_list[15]._value, columnVector);
    let d6 = math.multiply(transform_list[0].T, transform_list[1].T, transform_list[2].T, transform_list[3]._value, transform_list[11]._value, transform_list[12].T, transform_list[13]._value, transform_list[14].dT, transform_list[15]._value, columnVector);
    J[0][0] = d0[0][0];
    J[1][0] = d0[1][0];
    J[0][1] = d1[0][0];
    J[1][1] = d1[1][0];
    J[0][2] = d2[0][0];
    J[1][2] = d2[1][0];
    J[0][5] = d5[0][0];
    J[1][5] = d5[1][0];
    J[0][6] = d6[0][0];
    J[1][6] = d6[1][0];
  }
  else if (end_effector.name === "l_foot") {
    let d0 = math.multiply(transform_list[0].dT, transform_list[1].T, transform_list[2].T, transform_list[17]._value, transform_list[18].T, transform_list[19]._value, transform_list[20].T, transform_list[21]._value, columnVector);
    let d1 = math.multiply(transform_list[0].T, transform_list[1].dT, transform_list[2].T, transform_list[17]._value, transform_list[18].T, transform_list[19]._value, transform_list[20].T, transform_list[21]._value, columnVector);
    let d2 = math.multiply(transform_list[0].T, transform_list[1].T, transform_list[2].dT, transform_list[17]._value, transform_list[18].T, transform_list[19]._value, transform_list[20].T, transform_list[21]._value, columnVector);
    let d7 = math.multiply(transform_list[0].T, transform_list[1].T, transform_list[2].T, transform_list[17]._value, transform_list[18].dT, transform_list[19]._value, transform_list[20].T, transform_list[21]._value, columnVector);
    let d8 = math.multiply(transform_list[0].T, transform_list[1].T, transform_list[2].T, transform_list[17]._value, transform_list[18].T, transform_list[19]._value, transform_list[20].dT, transform_list[21]._value, columnVector);
    J[0][0] = d0[0][0];
    J[1][0] = d0[1][0];
    J[0][1] = d1[0][0];
    J[1][1] = d1[1][0];
    J[0][2] = d2[0][0];
    J[1][2] = d2[1][0];
    J[0][7] = d7[0][0];
    J[1][7] = d7[1][0];
    J[0][8] = d8[0][0];
    J[1][8] = d8[1][0];
  }
  else if (end_effector.name === "r_foot") {
    let d0 = math.multiply(transform_list[0].dT, transform_list[1].T, transform_list[2].T, transform_list[23]._value, transform_list[24].T, transform_list[25]._value, transform_list[26].T, transform_list[27]._value, columnVector);
    let d1 = math.multiply(transform_list[0].T, transform_list[1].dT, transform_list[2].T, transform_list[23]._value, transform_list[24].T, transform_list[25]._value, transform_list[26].T, transform_list[27]._value, columnVector);
    let d2 = math.multiply(transform_list[0].T, transform_list[1].T, transform_list[2].dT, transform_list[23]._value, transform_list[24].T, transform_list[25]._value, transform_list[26].T, transform_list[27]._value, columnVector);
    let d9 = math.multiply(transform_list[0].T, transform_list[1].T, transform_list[2].T, transform_list[23]._value, transform_list[24].dT, transform_list[25]._value, transform_list[26].T, transform_list[27]._value, columnVector);
    let d10 = math.multiply(transform_list[0].T, transform_list[1].T, transform_list[2].T, transform_list[23]._value, transform_list[24].T, transform_list[25]._value, transform_list[26].dT, transform_list[27]._value, columnVector);
    J[0][0] = d0[0][0];
    J[1][0] = d0[1][0];
    J[0][1] = d1[0][0];
    J[1][1] = d1[1][0];
    J[0][2] = d2[0][0];
    J[1][2] = d2[1][0];
    J[0][9] = d9[0][0];
    J[1][9] = d9[1][0];
    J[0][10] = d10[0][0];
    J[1][10] = d10[1][0];
  }
  return J;
}
function forwardK(end_effector) {
  let C = math.zeros([2, 1]);
  let columnVector = [[0],[0],[1]];
  if (end_effector.name === "l_hand") {
    let d0 = math.multiply(transform_list[0].T, transform_list[1].T, transform_list[2].T, transform_list[3]._value, transform_list[5]._value, transform_list[6].T, transform_list[7]._value, transform_list[8].T, transform_list[9]._value, columnVector);
    C[0][0] = d0[0];
    C[1][0] = d0[1];
  }
  else if (end_effector.name === "r_hand") {
    let d0 = math.multiply(transform_list[0].T, transform_list[1].T, transform_list[2].T, transform_list[3]._value, transform_list[11]._value, transform_list[12].T, transform_list[13]._value, transform_list[14].T, transform_list[15]._value, columnVector);
    C[0][0] = d0[0];
    C[1][0] = d0[1];
  }
  else if (end_effector.name === "l_foot") {
    let d0 = math.multiply(transform_list[0].T, transform_list[1].T, transform_list[2].T, transform_list[17]._value, transform_list[18].T, transform_list[19]._value, transform_list[20].T, transform_list[21]._value, columnVector);
    C[0][0] = d0[0];
    C[1][0] = d0[1];

  }
  else if (end_effector.name === "r_foot") {
    let d0 = math.multiply(transform_list[0].T, transform_list[1].T, transform_list[2].T, transform_list[23]._value, transform_list[24].T, transform_list[25]._value, transform_list[26].T, transform_list[27]._value, columnVector);
    C[0][0] = d0[0];
    C[1][0] = d0[1];
  }
  return C;
}
function rand_pose() {
  // Random translation values for the first 2 degrees of freedom
  //const translationDofs = Array.from({ length: 2 }, () => Math.random() * 10);
  const translationDofs = Array.from({ length: 2 }, () => 0);

  // Random rotation values for the remaining 9 degrees of freedom in radians
  const rotationDofs = Array.from({ length: 9 }, () => (Math.random() * 360 * Math.PI) / 180);

  // Combine translation and rotation into a single pose array
  const pose = [...translationDofs, ...rotationDofs];

  return pose;
}
function unit_test_J() {
  let ndofs = dof_list.length;
  for (let i = 0; i < 3; i++) {
    print('pose', i+1)
    q = rand_pose();
    let J_fd_lh = math.zeros([2, ndofs]);
    let J_fd_rh = math.zeros([2, ndofs]);
    let J_fd_lf = math.zeros([2, ndofs]);
    let J_fd_rf = math.zeros([2, ndofs]);
    set_joint_positions(q, 0);

    // call compute_jacobian - this is the analytical
    J_lh = compute_jacobian(name_to_transform.l_hand);
    J_rh = compute_jacobian(name_to_transform.r_hand);
    J_lf = compute_jacobian(name_to_transform.l_foot);
    J_rf = compute_jacobian(name_to_transform.r_foot);
    C_lh = forwardK(name_to_transform.l_hand);
    C_rh = forwardK(name_to_transform.r_hand);
    C_lf = forwardK(name_to_transform.l_foot);
    C_rf = forwardK(name_to_transform.r_foot);
    // for every dof
    for (let j = 0; j < 11; j++) {
      //take q(j) and perturb it a tiny amount
      let q_perturb = [...q];
      q_perturb[j] += .5*Math.PI/180;
      set_joint_positions(q_perturb, 0);

      //compute C again with new perturbation. 4 of them for each ee
      C_lh_perturb = forwardK(name_to_transform.l_hand);
      C_rh_perturb = forwardK(name_to_transform.r_hand);
      C_lf_perturb = forwardK(name_to_transform.l_foot);
      C_rf_perturb = forwardK(name_to_transform.r_foot);
      set_joint_positions(q, 0); //reset joints

      //take the difference between original C and perturbed C
      //this difference becomes the jth column of the finite difference Jacobian
      for (let k = 0; k < 2; k++) {
        J_fd_lh[k][j] = math.divide(math.subtract(C_lh[k], C_lh_perturb[k]),-.5*Math.PI/180)[0][0]; //difference and normalize
        J_fd_rh[k][j] = math.divide(math.subtract(C_rh[k], C_rh_perturb[k]),-.5*Math.PI/180)[0][0];
        J_fd_lf[k][j] = math.divide(math.subtract(C_lf[k], C_lf_perturb[k]),-.5*Math.PI/180)[0][0];
        J_fd_rf[k][j] = math.divide(math.subtract(C_rf[k], C_rf_perturb[k]),-.5*Math.PI/180)[0][0];  
      }
          
    }
    //compare each FD jacobian (4) to each analytical Jacobian (4)
    print('left hand');
    print("analytical:\t"+J_lh[0]+J_lh[1]);
    print("finite difference:\t"+J_fd_lh[0]+J_fd_lh[1]);
    print('right hand')
    print("analytical:\t"+J_rh[0]+J_rh[1]);
    print("finite difference:\t"+J_fd_rh[0]+J_fd_rh[1]);
    print('left foot')
    print("analytical:\t"+J_lf[0]+J_lf[1]);
    print("finite difference:\t"+J_fd_lf[0]+J_fd_lf[1]);
    print('right foot')
    print("analytical:\t"+J_rf[0]+J_rf[1]);
    print("finite difference:\t"+J_fd_rf[0]+J_fd_rf[1]);
  }
}

function draw_character() {
  bone_segments.forEach((seg) => {
    seg.draw();
  });

  collision_candidates.draw();

  // draw CoM xy
  push();
  noStroke();
  fill(0, 128, 0);
  let com = get_init_CoM_xy(false);
  circle(com[0], com[1], 0.1);
  pop();
}

function setup_character() {
  // Reset the data structure
  transform_list = [];
  name_to_transform = {};
  dof_list = [];

  // Define the character
  let base = new Translation("base_x", "x")
    .add(new Translation("base_y", "y"))
    .add(new Hinge("base_r"));

  let torso = base.add(new Fixed(`root_to_torso`, 0, 0.25));

  torso.add(new Fixed(`torso_to_head`, 0, 0.3));

  ["l", "r"].forEach((d) => {
    let sx = d === "l" ? 1.0 : -1.0;
    torso
      .add(new Fixed(`${d}_torso_to_shoulder`, sx * 0.1, 0.15))
      .add(new Hinge(`j_${d}_shoulder`))
      .add(new Fixed(`${d}_upperarm`, sx * 0.3, 0.0))
      .add(new Hinge(`j_${d}_elbow`))
      .add(new Fixed(`${d}_lowerarm`, sx * 0.3, 0.0))
      .add(new Fixed(`${d}_hand`, 0.0, 0.0));
  });

  ["l", "r"].forEach((d) => {
    let sx = d === "l" ? 1.0 : -1.0;
    base
      .add(new Fixed(`${d}_root_to_hip`, sx * 0.1, -0.2))
      .add(new Hinge(`j_${d}_hip`))
      .add(new Fixed(`${d}_upperleg`, 0.0, -0.4))
      .add(new Hinge(`j_${d}_knee`))
      .add(new Fixed(`${d}_lowerleg`, 0.0, -0.4))
      .add(new Fixed(`${d}_foot`), 0.0, 0.0);
  });

  // Build auxiliary data structures
  transform_list.forEach((tr) => {
    name_to_transform[tr.name] = tr;
    if (tr.num_dofs() > 0) {
      dof_list.push(tr);
    }
  });

  // Build dependent dofs
  transform_list.forEach((tr) => {
    tr.dependent_dofs = tr.parent
      ? new Set(tr.parent.dependent_dofs)
      : new Set(); //inherit the parent dofs or create new set if no parents
    if (tr.num_dofs() > 0) {
      tr.dependent_dofs.add(tr);
    }
  });

  // print(transform_list);
  // print(name_to_transform);
  // print(dof_list);
}

function keyPressed() {
  if (keyCode == ENTER && !physics_on) {
    physics_on = true;
    init_phys_state();
    physics_timer = physics_duration * FRAMERATE;
  }
}

function mousePressed() {
  if (physics_on || remaining_drags <= 0) {
    dragging_idx = -1;
    return;
  }

  let x = math.divide(mouseX - x_offset_draw, g_s);
  let y = math.divide(mouseY - y_offset_draw, g_s);

  // find the closest end effector to mouse
  // if mouse is close enough to an end effector, start a circle tracking the mouse
  // do not naively update the target position of the end effector to the mouse position
  // cap target position to be within the circle around the end effector

  dragging_idx = -1;
  max_dist = 0.2;
  for (let i = 0; i < ees.length; i++) {
    let p_ee = ees[i].global_position();
    let mouse_pos = [x, y];
    let diff = math.subtract(mouse_pos, p_ee);
    if (math.norm(diff) < max_dist) {
      dragging_idx = i;
      max_dist = math.norm(diff);
    }
  }
}

function mouseReleased() {
  if (physics_on || remaining_drags <= 0 || dragging_idx < 0) {
    return;
  }
  dragging_idx = -1;
  remaining_drags -= 1;
}

function mouseDragged() {
  if (physics_on || remaining_drags <= 0) {
    dragging_idx = -1;
    return;
  }

  let x = math.divide(mouseX - x_offset_draw, g_s);
  let y = math.divide(mouseY - y_offset_draw, g_s);

  if (dragging_idx >= 0) {
    let p_ee = ees[dragging_idx].global_position();
    let diff = math.subtract([x, y], p_ee);
    let alpha = math.norm(diff);
    let max_dist = 0.1;
    let diff_capped = math.multiply(diff, math.min(1.0, max_dist / alpha));

    let target = math.add(p_ee, diff_capped);

    // push targets out of collision
    collision_output = collisionCheck(target, obstacles);
    if (collision_output[0] >= 0) {
      c_depth = collision_output[0];
      c_dir = collision_output[1];
      target = math.add(target, math.multiply(c_depth, c_dir));
    }

    targets[dragging_idx] = target;
  }
}

/** SDF of all scene obstacles.
 * @param {2-array} p 2D position to query SDF at.
 * @return {number,object} [d,o] Distance, d, to closest obstacle, o.
 */
// function distanceO(p, obstacles) {
//   let result = [Infinity, this];
//   for (let i = 0; i < obstacles.length; i++) {
//     let cur = obstacles[i].distanceO(p);
//     result = SDF.opUnion(result, cur);
//   }
//   return result;
// }

function normalize(v) {
  let v_mag = math.norm(v);
  if (v_mag > 1e-6) {
    return math.divide(v, v_mag);
  }
  return v; // return (0, 0)
}

function collisionCheck(p_vec, obstacles) {
  // Get the signed distance to the nearest obstacle.
  let distO = distanceO(p_vec);
  let obstacle = distO[1];

  let particle_radius = 0.1; // should we instead assume 0 radius?
  let d = distO[0] - particle_radius;
  let nvec = (obstacle.normal(createVector(p_vec[0], p_vec[1]))).normalize();
  let n = [nvec.x, nvec.y];
  let particlePenetrationDepth = 0;

  if (d < 0) {
    particlePenetrationDepth = max(particlePenetrationDepth, abs(d));
  }
  return [particlePenetrationDepth, n];
}

