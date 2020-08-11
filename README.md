# [Pepper humanoid](https://www.softbankrobotics.com/emea/pepper) robot module for [`mc_rtc`](https://jrl-umi3218.github.io/mc_rtc/)

This package contains a software representation, or a robot module, of [Pepper humanoid robot](https://www.softbankrobotics.com/emea/pepper) platform. This robot module is used by [`mc_rtc`](https://jrl-umi3218.github.io/mc_rtc/) framework to realize control of Pepper robot in simulation or in experiments with real platform.

### Required dependencies

* [`mc_rtc`](https://jrl-umi3218.github.io/mc_rtc/)
* [`pepper_description`](https://gite.lirmm.fr/softbankrobotics/pepper_description)

### Build instructions

``` bash
git clone git@gite.lirmm.fr:multi-contact/mc_pepper.git
cd mc_pepper
mkdir build
cd build
cmake ..
make
sudo make install
```

### Robot devices

This robot module allows to interact with some robot [`devices`](src/devices) from `mc_rtc` controller, namely:

* `Speakers` - command a word or a sentence to say
* `TouchSensor` - detect when mobile base bumper is touched
* `VisualDisplay` - set tablet screen image

### Robot module types

Depending on the use-case, users might prefer to use one of the following implemented types of the Pepper robot module:
* `Pepper` - moving base, hand joints included (default module)
* `PepperFixed` - fixed base, hand joints included
* `PepperNoHands` - moving base, hand joints not included
* `PepperFixedNoHands` - fixed base, hand joints not included
* `PepperExtraHardware` - moving base, hand joints and extra hardware included
* `PepperExtraHardwareNoHands` - moving base, hand joints not included, extra hardware included
* `PepperFixedExtraHardwareNoHands` - fixed base, hand joints not included, extra hardware included


## Example of custom tasks and constraints

In `mc_rtc` controller two main elements for robot control are
* **Tasks** - objectives, what robot should do the best it can
* **Constraints** - limits, that robot should always respect

Many tasks and constraints are already implemented in `mc_rtc`. For instance `PostureTask`, `CoMTask`, `EndEffectorTask`, `KinematicsConstraint`, ` ContactConstraint` etc. However, in some cases it might be desirable to design and implement **new custom tasks or constraints** not yet implemented in `mc_rtc`. Such new tasks and constraint might be specific to a robot, use-case or research topic.

In this repository, we provide an example of a custom `CoMRelativeBodyTask`, that allows to specify desired Pepper CoM target relative to the robot mobile base frame (as opposed to world frame in `mc_rtc CoMTask`). Implementation of this custom task can be found in the [`tasks`](src/tasks) folder.

An implementation of a custom `BoundedAccelerationConstr` constraint, to impose acceleration bounds for Pepper mobile base, can be found in the [`constraints`](src/constraints) folder.

How these custom tasks and constraints are loaded and used in a sample `mc_rtc` controller can be see in [PepperFSMController](https://gite.lirmm.fr/mc-controllers/pepperfsmcontroller).

**In an analogous way, many other novel tasks and constraints can be implemented and tested**

### Next steps

Now that `mc_pepper` is successfully installed, you can use [`PepperFSMController`](https://gite.lirmm.fr/nana/pepperfsmcontroller) project as a staring point or an example on how to write your own `mc_rtc` controller for Pepper robot.

Once your controller is ready to be tested, you can use [`mc_naoqi`](https://gite.lirmm.fr/multi-contact/mc_naoqi) interface to run and visualize your controller in simulation or to run your controller on a real Pepper humanoid robot platform.
