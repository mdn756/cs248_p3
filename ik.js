// TODO: student code
function take_IK_step(end_effectors, targets) {
    // targets being a list of 2-arrays
    // end_effectors being a list of Transformation objects
    // return the gradient of the IK residual, a 1d array of length ndofs
    let g = math.zeros([q.length]);
    for (let i = 0; i < ees.length; i++) {
        let ee = ees[i];
        let pos = ee.global_position();
        let r = targets[i];
        let cost_der = math.subtract(pos, r);
        // cost_der = transpose(cost_der);

        let derivative = math.multiply(cost_der, compute_jacobian(ee));
        // print("derivative: "+derivative);
        // print("g: "+g);
        g = math.add(g, derivative);
      // global
      // print("end_effectors: "+end_effectors[i].global_position());
    }
  
    // print("targets: "+targets);
    return g;
  }