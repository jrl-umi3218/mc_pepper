# [Pepper humanoid robot](https://www.softbankrobotics.com/emea/pepper) [`mc_rtc`](https://jrl-umi3218.github.io/mc_rtc/) module

This package contains a software representation, or robot module, of [Pepper humanoid robot](https://www.softbankrobotics.com/emea/pepper) platform. This robot module is used by [`mc_rtc`](https://jrl-umi3218.github.io/mc_rtc/) framework to realize control of Pepper robot in simulation or in experiments with real platform.

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

### Next steps

Now that `mc_pepper` is successfully installed, you can use [`PepperFSMController`](https://gite.lirmm.fr/nana/pepperfsmcontroller) project as a staring point or an example on how to write your own `mc_rtc` controller for Pepper robot.

Once your controller is ready to be tested, you can use [`mc_naoqi`](https://gite.lirmm.fr/multi-contact/mc_naoqi) interface to run and visualize your controller in simulation or to run your controller on a real Pepper humanoid robot platform.

### Robot module types

* `Pepper` - moving base, hand joints included (default module)
* `PepperFixed` - fixed base, hand joints included
* `PepperNoHands` - moving base, hand joints not included
* `PepperFixedNoHands` - fixed base, hand joints not included
* `PepperExtraHardware` - moving base, hand joints and extra hardware included
* `PepperExtraHardwareNoHands` - moving base, hand joints not included, extra hardware included
* `PepperFixedExtraHardwareNoHands` - fixed base, hand joints not included, extra hardware included
