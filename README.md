# Automatic Testing for Physical Changes

This is the final project for Formal Methods for CPS & Robots at the Unversity of Virginia spring 2019. For this project we wanted to find behavioural differences in robots due to hardware changes. The behavioural difference would be due to the complex interactions and design assumptions made between hardware and software. To generate tests which would highlight these difference we generated a set of constraints to place obstacles in hte environment such that the only one of the robots would likely sense the obstacle. The constraints were solved using [Z3](https://github.com/Z3Prover/z3) a theorem prover from Microsoft Research. The robot we tested was a [Clearpath Husky robot](https://www.clearpathrobotics.com/). Clearpath robotics has open source code for both control and simulation of their Husky robots. We were able to simulate hardware changes by changing the attributes of the laser scanners configuration file.

## Project Sections

This project contains a number of sections; each is listed below:

* [Documents](./docs/) - Contains other documentation which was generated during the design and implmentation process.
* [Miscellaneous](./misc/) - Contains videos and images from testing as well as edited images and videos used in the final presentation and video.
* [Presentation](./presentation) - Contains the final presentation.
* [Report](./report) - Contains the final report.
* [Source](./src) - Contains the final source code for both the constraints and simulation.
* [Final Video](./video) - Contains the final video.

## Video

[Todo](todo)