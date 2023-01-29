# Krust

## What is it?

Krust is a tiny inverse kinematics library using gradient descent to optimize a kinematic chain w/ basic collision avoidance. It only supports revolute joints at the moment.

It is meant to be used as a webassembly package for a demo I made [here](https://www.grippy.app).

## How to use:

In the krust directory, run:

```bash
wasm-pack build --target web
```

Copy the pkg folder to the src folder on your webserver. The webassembly module can now be loaded like any other module.

