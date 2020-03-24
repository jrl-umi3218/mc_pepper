## This is a Pepper robot module for `mc_rtc` control framework

This module uses `pepper_description` ROS package

The following types of modules are currently implemented for user's convenience

* `Pepper` - moving base, hand joints included (default module)
* `PepperFixed` - fixed base, hand joints included
* `PepperNoHands` - moving base, hand joints not included
* `PepperFixedNoHands` - fixed base, hand joints not included
* `PepperExtraHardware` - moving base, hand joints and extra hardware included
* `PepperExtraHardwareNoHands` - moving base, hand joints not included, extra hardware included
* `PepperFixedExtraHardwareNoHands` - fixed base, hand joints not included, extra hardware included

## Extra info

Note that at the moment this module works with `topic/custom_sensors_devices` branch of `mc_rtc`
