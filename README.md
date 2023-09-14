## What is this program for?
Examples to use UR rtde library.

## What are the files?
- `rtde` folder: the rtde library from UR company.
- `only_move_example.py`: an example only to move robotic arm.
- `only_digital_output_example.py`: an example only to change the values of standard digital outputs.
- `move_digital_output_example.py`: an example to move robotic arm and change the values of standard digital outputs together.
- `control_loop_configuration.xml`: config file for `rtde` library, including the variables that we need to write/change in the UR robotic arm.
- `record_configuration.xml`: config file for `rtde` library, including the variables that we need to read from the UR robotic arm.

## other information

### An email from UR (Universal Robots) company
UR company sent us an email, which also includes some documentations and examples to use the `rtde` library:

<blockquote>
Here you can find a good documentation about our rtde interface. I can recommend you to go through step by step on this page.

https://www.universal-robots.com/articles/ur/interface-communication/real-time-data-exchange-rtde-guide/

You can find input and output options for rtde xml files in “FIELD NAMES AND ASSOCIATED TYPES” section.

Also we have a github repo that you can find rtde examples.

https://github.com/UniversalRobots/RTDE_Python_Client_Library

Let me know if you need anything else.

Regards.

</blockquote>


### Another rtde example repository
UR company recommended this repository to us:
> https://github.com/KeepTheBeats/URRobots_Python_RTDE

I have not read it, but forked it, in case that we may need it in the future.