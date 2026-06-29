# MissionSelector LUA script

This script will select and load one of three mission files whenever the AUX FUNCTION switch for Mission Reset (24) changes position, as long as the vehicle is **not in AUTO**. This allows easy, at the field selection of missions and, importantly, lets you change the active mission **in flight** from a non-AUTO mode (e.g. Loiter). MissionH.txt, MissionM.txt, or MissionL.txt mission files in the SD card root are loaded for the High/Mid/Low switch position. The matching mission is also loaded once at startup so the loaded mission mirrors the switch position.

If the switch is moved while the vehicle is in **AUTO** (the mission is being flown), the mission is left unchanged and the user is warned. The new selection is applied automatically as soon as the vehicle leaves AUTO, so the loaded mission always mirrors the switch position whenever it is safe to change it.

The basic AUX function of resetting the mission pointer to the first waypoint is unaltered when the switch is moved to the high position, as previous.

If the AUX FUNCTION rc switch is not configured, the script is not active.

If the file selected by the switch position is not in the root SD directory, nothing happens, otherwise the current mission will be cleared and the designated mission file will be loaded and the user notified of the mission change.

If the file is available but is not the correct format, the script reports the error and leaves the existing mission unchanged (or empty if the file ends part way through an item); the script keeps running so subsequent switch changes still work.

So a user can install the script and if the switch is configured and a file exists for selection it will function, but either can be missing without causing messaging to the user.
