Overview:

Autonomous vehicles are all the rage. Waymo, Cruise, .... you name it. It’s an exciting time, and
we don’t want to miss out on the action. What could be cooler than building your own
autonomous vehicle?
For the final project of 424 you will work on a team to program the brains of an autonomous
RC/toy car. If it’s autonomous, then of course RC (remote control) loses its meaning, but if we
say toy car then it might be ambiguous. So, we’ll say RC car often just because it implies a
small toy car with electronics. First you’ll get some motor control implemented for project 3.
The remaining tasks for motor feedback and lane keeping will be the final project.
To make motor control happen for project 3, you can use the code from last year as a starting
point: link [You can literally start from their scripts, just be sure to cite your sources]. For motor
control, you will only want to look at projects that list a BeagleBone in their hardware (example).
Any of the scripts from the BeagleBone projects take an existing Python script meant for a
Raspberry Pi attached to an RC car and modify it to work with the Beaglebone AI-64 (BBAI64)
attached to some RC cars.
Your job for project 3 is to use a Raspberry Pi 5 to provide speed and steering control of the
RC car. Although the existing Python script is for the Pi already, the work done on Hackster
with BeagleBones is more true to what the RC cars we are working with expect. So, the
existing Python script is only helpful in that it shows some of the GPIO functions in action that
you will use for the Pi, but the parameters might need to be more like the Hackster BeagleBone
projects.
This is a team submission. Each team should have 1 demo and 1 submission for Canvas
