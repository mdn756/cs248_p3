// TODO: student code
function take_IK_step(end_effectors, targets) {
  // targets being a list of 2-arrays
  // end_effectors being a list of Transformation objects
  // return the gradient of the IK residual, a 1d array of length ndofs
  let g = math.zeros([q.length]);
  for (let i = 0; i < end_effectors.length; i++) {
      let ee = end_effectors[i];
      let pos = ee.global_position();
      let r = targets[i];
      let cost_der = math.subtract(pos, r);
      // cost_der = transpose(cost_der);

      let derivative = math.multiply(cost_der, compute_jacobian(ee));
      // print("derivative: "+derivative);
      // print("g: "+g);
      g = math.add(g, derivative);
  }

  // print("targets: "+targets);
  return g;
}

function calculate_cost(end_effectors, targets) {
  let c = 0
  for (let i = 0; i < end_effectors.length; i++) {
    let ee = end_effectors[i];
    let pos = ee.global_position();
    let r = targets[i];
    let cost_der = math.subtract(pos, r);
    let two_norm = Math.sqrt(cost_der[0]*cost_der[0]+cost_der[1]*cost_der[1]);
    let two_norm_sq = two_norm * two_norm;
    c += two_norm_sq;
  }
  return 0.5*c;
}

function IK_unit_test() {
  let fd = math.zeros([11]);
  for (let i = 0; i < 3; i++) {
    print('pose', i+1)
    q = rand_pose();
    r = rand_pose();
    set_joint_positions(q, 0);

    // call compute_jacobian - this is the analytical
    let analytical_gradient = take_IK_step(ees, targets);
    print("analytical_gradient: "+analytical_gradient);

    // cost before perturbing
    let cost_bf = calculate_cost(ees, targets); 
    for (let j = 0; j < 11; j++) {
      //take q(j) and perturb it a tiny amount
      let q_perturb = [...q];
      q_perturb[j] += .5*Math.PI/180;
      set_joint_positions(q_perturb, 0);
      // cost after perturbing
      let cost_af = calculate_cost(ees, targets);
      let fd_der = (cost_bf - cost_af) / (-.5*Math.PI/180); //difference and normalize
      fd[j] = fd_der;          
      set_joint_positions(q, 0); //reset joints
    }
    print("fd: "+ fd);
  }
}
