# Autoware Launcher Plugin File Format

The plugin file is yaml file. Currently, there are the following versions.
* Plugin Version 0.1

# Plugin Version 0.1

The file is the following dictionary.

| Key      | Type   | Value/Comment |
|----------|--------|---------------|
| format   | string | Fixed string: "Autoware Launcher Plugin Version 0.1" |
| launch   | string | roslaunch xml file path |
| args     | [Group Definition List](#group_definition) | Arguments |
| exts     | [Group Definition List](#group_definition) | Extended Information |
| children | [Node Rule List](#node_rule) | Children Rules |


## <a id="group_definition">Group Definition</a>

Group Definition is dictionary.

| Key    | Type   | Value/Comment |
|--------|--------|---------------|
| text   | string | GUI Frame Text (Optional) |
| view   | string | GUI Frame Type |
| data   | [Field Definition](#field_definition)<br>[Field Definition List](#field_definition) | Data for GUI Frame |

## <a id="field_definition">Field Definition</a>

Field Definition is dictionary.

| Key    | Type   | Value/Comment |
|--------|--------|---------------|
| type   | string | Enum (bool, str, int, real) |
| list   | string | Enum (null, space, yaml) |
| name   | string | Field Name (For args, set same value as roslaunch) |
| cfgkey | string | Hidden Fields (Do not set in plufin file) |

## <a id="node_rule">Node Rule</a>

T.B.D.

## <a id="child_definition">Plugin Reference</a>

T.B.D.



