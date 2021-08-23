#pragma once

namespace planner {
  enum State { INPUT_CHANGED, REGENERATE, DO_NOTHING };
  enum Motion { PURE_YAW, PURE_Z, ZERO, NAVIGATION };
}
