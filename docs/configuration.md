# Configuration Guide

## Overview

This document explains how to configure YAML files in the config directory for the parameter server. These files define parameters for the parameter server node, including their default values, types, descriptions, and constraints.

## File Location

Place YAML files in the config directory of your ROS 2 package. The node will parse all YAML files in this directory and its subfolders. If the package is built using `colcon build --symlink-install`, you can modify the files in the source directory without rebuilding the package.

## YAML File Structure

Each YAML file can contain multiple parameter groups. Each parameter group contains multiple parameters with attributes:

- `default`: Specifies the default value. Use a sequence (`[]`) for arrays.
- `type`: Defines the parameter type (`int`, `double`, `bool`, `string`).
- `description`: (Optional) Describes the parameter.
- `min`: (Optional) Minimum value for `int` and `double` types.
- `max`: (Optional) Maximum value for `int` and `double` types.

The parameter server node will load all parameters from the YAML files and expose them as ROS 2 parameters. The parameters can be accessed and modified using the ROS 2 parameter client API. They are named using the following convention: `<parameter_group>.<parameter_name>`.

### Example `parameters.yaml`

```yaml
Parameter_group_1:
  Parameter1:
    default: 5
    min: 0
    max: 10
    type: int
    description: "An integer parameter"

  Parameter2:
    default: 3.14
    min: 0.0
    max: 10.0
    type: float

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
    type: float

  ArrayParameter3:
    default: [true, false, true]
    type: bool

  ArrayParameter4:
    default: ["foo", "bar", "baz"]
    type: string
```

