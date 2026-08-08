from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.parameter_descriptions import ParameterValue

DEFAULT_POSE_BY_WORLD = {
    'living_room': {'x': 0.0, 'y': 1.5, 'yaw': 0.0},
    'obstacle_arena': {'x': 0.0, 'y': 0.0, 'yaw': 0.0},
    'small_house': {'x': 0.0, 'y': 0.0, 'yaw': 0.0},
}

DEFAULT_FALLBACK_POSE = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}


def _default_axis_expression_str_parts(world_arg_name, axis):
    world_lc = LaunchConfiguration(world_arg_name)
    parts = []
    open_parens = 0
    for world_name, pose in DEFAULT_POSE_BY_WORLD.items():
        parts.append(f"({pose[axis]} if '")
        parts.append(world_lc)
        parts.append(f"' == '{world_name}' else ")
        open_parens += 1
    parts.append(str(DEFAULT_FALLBACK_POSE[axis]))
    parts.append(')' * open_parens)
    return parts


def resolve_axis(world_arg_name, axis, user_override_arg_name=None):
    default_parts = _default_axis_expression_str_parts(world_arg_name, axis)

    if user_override_arg_name is None:
        return PythonExpression(default_parts)

    override_lc = LaunchConfiguration(user_override_arg_name)
    parts = ["float('", override_lc, "') if '", override_lc, "' != '' else ("]
    parts.extend(default_parts)
    parts.append(")")
    return PythonExpression(parts)


def resolve_pose(world_arg_name='world',
                  x_override_arg_name='spawn_x',
                  y_override_arg_name='spawn_y',
                  yaw_override_arg_name='spawn_yaw'):
    return {
        'x': resolve_axis(world_arg_name, 'x', x_override_arg_name),
        'y': resolve_axis(world_arg_name, 'y', y_override_arg_name),
        'yaw': resolve_axis(world_arg_name, 'yaw', yaw_override_arg_name),
    }


def as_float_param(expression):
    return ParameterValue(expression, value_type=float)
