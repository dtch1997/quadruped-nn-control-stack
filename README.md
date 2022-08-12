# Modular Control Architecture

This repo aims to design a unified and modular control architecture that enables easy interchanging of high-level controllers and low-level hardware. 

![control_architecture](docs/modular_control_arch.png)

## Roadmap

- [x] Design interfaces
- [x] Implement example dummy hardware and controller
- [x] Implement Unitree hardware 
- [ ] Implement example controller
- [ ] Verify that full control setup works on hardware

## Quickstart

### Install Dependencies (Docker)
This repository comes bundled with a Docker development container that bundles the dependencies necessary to build the project. It can be opened using VSCode Remote Containers extension

### Build Executables

```
mkdir build
cd build
cmake ..
make 
```

`examples/fake_application` demonstrates how to implement a simple application with dummy controller and hardware. 
