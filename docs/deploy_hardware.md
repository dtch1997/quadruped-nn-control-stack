# Guide

This document explains how to compile binary in the dev container and then transfer the working version to the Unitree A1 hardware. 

## Get the Deployment Workspace

First, we need to confirm the deployment workspace. For example:
```
/home/unitree/nn_loco_ws/nn_locomotion_unitree
```

This workspace will contain the ```build``` folder that has all binaries

## Set the Dev Container Workspace Mount Point

We will set up a workspace path inside a Docker development container that matches the deployment workspace **exactly**. 

In `.devcontainer/devcontainer.json`, set workspaceFolder and workspaceMount to match the deployment workspace **exactly**

```
  // Modify the workspaceFolder to match deployment workspace exactly
  "workspaceFolder": "/home/unitree/nn_loco_ws/nn_locomotion_unitree",
  "workspaceMount": "source=${localWorkspaceFolder},target=/home/unitree/nn_loco_ws/nn_locomotion_unitree,type=bind"
```

## Open Dev Container

This is easiest in VSCode. You will need VSCode Remote Containers extension.

From your host machine, run: 

```
code <workspace_root>
```

If you have VSCode remote container extensions installed, it should prompt you to open the workspace in a container.

If you missed the chance to accept the prompt, you can always use the Remote-Containers: Reopen in Container command (by default press ctrl+shift+p, and then type in the command) to open the workspace in the container.

## Compile binary inside Dev Container

Inside dev container, do the usual procedure to compile binaries 

```
# From top level directory
mkdir build && cd build
cmake ..
make
```

## Transfer binary to Hardware

The newly created files in the ```build``` folder should be visible from the host machine. Now, from the host machine, copy the binary and any libraries it depends on into the hardware. The deployment workspace path must match the build workspace path **exactly** or the binary will not be able to find its associated libraries. 

Finally, from the hardware, the binary can be run directly :)