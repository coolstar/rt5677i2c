# rt5677
Realtek ALC 5677 I2C Codec driver

Supports:
* Jack Detection (Only detects whether headphones are plugged in. Assumes Headphones have microphone).
* Headphone output
* Sleep/Wake

Note:
* Intel SST proprietary drivers do NOT have documented interfaces, so this driver will not work with them.
* Using this driver on chromebooks with this audio chip will require using CoolStar SST Audio

Tested on Google Chromebook Pixel 2 LS (2015)