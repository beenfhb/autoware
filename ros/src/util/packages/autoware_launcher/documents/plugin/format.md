# Autoware Launcher Plugin File Format

The plugin file is yaml file. Currently, there are the following versions.
* Plugin Version 0.1 (experimental)

# Plugin Version 0.1

## Plugin File
The file is the following dictionary.

#### Node Type

| Key      | Type   | Value/Comment |
|----------|--------|---------------|
| format   | string | Fixed string: "Autoware Launcher Plugin Version 0.1" |
| exts     | [Extended Data List](#extended_data) | Extended Data Definitions |
| rules    | [Children Rule List](#children_rule) | Children Rule Definitions |

#### Leaf Type
| Key      | Type   | Value/Comment |
|----------|--------|---------------|
| format   | string | Fixed string: "Autoware Launcher Plugin Version 0.1" |
| rosxml   | string | roslaunch xml file path |
| exts     | [Extended Data List](#extended_data) | Extended Data Definitions |
| args     | [Argument Data List](#argument_data) | Argument Data Definitions |


## <a id="argument_data">Argument Data</a> / <a id="extended_data">Extended Data</a>

dictionary

| Key    | Type          | Value/Comment |
|--------|---------------|---------------|
| name   | string        | Data Name (For args, roslaunch arg tag) |
| type   | string        | GUI Frame Type |
| list   | string / null | Data for GUI Frame |

## =====

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



