class Transformation {
  constructor(name = null) {
    transform_list.push(this);
    this.name = name;
    this.parent = null;
    this.children = [];
    this.dependent_dofs = new Set();
  }

  add(child) {
    child.parent = this;
    this.children.push(child);
    return child;
  }

  global_transform() {
    return transform_between(null, this);
  }

  global_position() {
    return math.flatten(
      math.subset(this.global_transform(), math.index([0, 1], 2))
    );
  }

  local_transform() {
    throw "not implemented";
  }

  local_derivative() {
    throw "not implemented";
  }

  num_dofs() {
    throw "not implemented";
  }

  get_dof() {
    throw "not implemented";
  }

  set_dof(value = 0.0) {
    throw "not implemented";
  }

  toString() {
    return `${this.constructor.name}(${this.name})`;
  }
}

class Fixed extends Transformation {
  constructor(name, tx = 0.0, ty = 0.0) {
    super(name);
    this._value = math.identity([3]);
    this._value = math.subset(this._value, math.index([0, 1], 2), [[tx], [ty]]);
  }

  num_dofs() {
    return 0;
  }

  local_transform() {
    return this._value;
  }
}

class Translation extends Transformation {
  constructor(name, axis, value = 0.0) {
    super(name);
    this._value = value;
    this.axis = axis;

    if (axis === "x") {
      this.axis_index = 0;
    } else if (axis === "y") {
      this.axis_index = 1;
    }
    this.T = math.identity([3]);
    this.dT = math.zeros([3, 3]);
  }

  num_dofs() {
    return 1;
  }

  get_dof() {
    return this._value;
  }

  set_dof(value = 0.0) {
    this._value = value;
  }

  local_transform() {
    if (this.axis === "x") {
      this.T[0][2] = this._value;
    } else if (this.axis === "y") {
      this.T[1][2] = this._value;
    }
    return this.T;
  }

  local_derivative() {
    if (this.axis === "x") {
      this.dT[0][2] = 1;
    } else if (this.axis === "y") {
      this.dT[1][2] = 1;
    }
    return this.dT;
  }
}

class Hinge extends Transformation {
  constructor(name, theta = 0.0) {
    super(name);
    this._theta = theta;
    this.T = math.identity([3]);
    this.dT = math.zeros([3, 3]);
  }

  num_dofs() {
    return 1;
  }

  get_dof() {
    return this._theta;
  }

  set_dof(value = 0.0) {
    this._theta = value;
  }

  local_transform() {
    const cosTheta = Math.cos(this._theta);
    const sinTheta = Math.sin(this._theta);

    this.T = [
      [cosTheta, -sinTheta, 0],
      [sinTheta, cosTheta, 0],
      [0, 0, 1]
    ];
    return this.T;
  }

  local_derivative() {
    const cosTheta = Math.cos(this._theta);
    const sinTheta = Math.sin(this._theta);

    this.dT = [
      [-sinTheta, -cosTheta, 0],
      [cosTheta, -sinTheta, 0],
      [0, 0, 0]
    ];
    return this.dT;
  }
}