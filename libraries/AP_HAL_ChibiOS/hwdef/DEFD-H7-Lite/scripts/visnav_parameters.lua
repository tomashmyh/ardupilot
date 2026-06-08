-- Adding parameters of Visual Navigation via lua script

-- The key is persistent in storage and must be unique. Value 0..200.
local PARAM_TABLE_KEY = 158

-- Create a parameter table
assert(param:add_table(PARAM_TABLE_KEY, "VISNAV_", 6), 'could not add param table')

-- Create parameters.
-- The param indexes (2nd argument) must be between 1 and 63.
-- All added parameters are floats, with the given default value (4th argument).
assert(param:add_param(PARAM_TABLE_KEY, 1, 'CTRL_CH', 10.0), 'could not add CTRL_CH')
assert(param:add_param(PARAM_TABLE_KEY, 2, 'CAM_ANGLE', 90.0), 'could not add CAM_ANGLE')
assert(param:add_param(PARAM_TABLE_KEY, 3, 'R2D2_SYNC', 0), 'could not add R2D2_SYNC')
assert(param:add_param(PARAM_TABLE_KEY, 4, 'GPS_SYNC', 1), 'could not add GPS_SYNC')
assert(param:add_param(PARAM_TABLE_KEY, 5, 'GCS_SYNC', 1), 'could not add GCS_SYNC')
assert(param:add_param(PARAM_TABLE_KEY, 6, 'SNLN_SYNC', 0), 'could not add SNLN_SYNC')
