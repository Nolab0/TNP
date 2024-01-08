# TNP

## Authors

Jean FECHTER \
Thibault BOUTET

## Build

```bash
$ mkdir build
$ cd build
$ cmake ..
$ make
```

## Run

```bash
$ ./ransac [PATH_TO_OBJ_FILE]
```

PATH_TO_OBJ_FILE: path to the object file
!! Make sure that the .obj file contains the normals for each point !!

### Example

```bash
$ ./ransac ../data/church.obj
```