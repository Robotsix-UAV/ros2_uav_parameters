## Configuration Guide

THIS IS A TEST
AND MORE
### Overview

This document explains how to configure YAML files in the `config` directory for the parameter server. These files define parameters for the parameter server node, including their default values, types, descriptions, and constraints.

### File Location

Place YAML files in the `config` directory of your ROS 2 package. The node will parse all YAML files in this directory and its subfolders. If the package is built using `colcon build --symlink-install`, you can modify the files in the source directory without rebuilding the package.

### YAML File Structure

Each YAML file can contain multiple parameter groups. Each parameter group contains multiple parameters with attributes:

- `default`: Default value of the parameter.
- `type`: Data type (`int`, `double`, `bool`, `string`).
- `description`: (Optional) Description of the parameter.
- `constraints_description`: (Optional) Additional constraints on the parameter.
- `min`: (Optional) Minimum value for `int` and `double` types.
- `max`: (Optional) Maximum value for `int` and `double` types.

### Example `parameters.yaml`

```yaml
Parameter_group_1:
  Parameter1:
    default: 5
    min: 0
    max: 10
    type: int
    description: "An integer parameter"
    constraints_description: "Must be between 0 and 10"

  Parameter2:
    default: 3.14
    min: 0.0
    max: 10.0
    type: double

  Parameter3:
    default: true
    type: bool

  Parameter4:
    default: "hello"
    type: string

Parameter_group_2:
  ArrayParameter1:
    default: [1, 2, 3]
    type: int

  ArrayParameter2:
    default: [1.1, 2.2, 3.3]
    type: double

  ArrayParameter3:
    default: [true, false, true]
    type: bool

  ArrayParameter4:
    default: ["foo", "bar", "baz"]
    type: string
```

### Attributes

- **default**: Specifies the default value. Use a sequence (`[]`) for arrays.
- **type**: Defines the parameter type (`int`, `double`, `bool`, `string`).
- **description**: (Optional) Describes the parameter.
- **constraints_description**: (Optional) Additional constraints or information.
- **min**: (Optional) Minimum value for `int` and `double` types.
- **max**: (Optional) Maximum value for `int` and `double` types.
