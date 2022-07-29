![GitHub](https://img.shields.io/github/license/Yessir120/HolonomicLib?logo=Github)
![GitHub all releases](https://img.shields.io/github/downloads/Yessir120/HolonomicLib/total?logo=Github)
![Website](https://img.shields.io/website?down_color=lightred&down_message=offline&label=api&logo=github&up_color=green&up_message=online&url=https%3A%2F%2Fyessir120.github.io%2FHolonomicLib%2Fhtml%2Findex.html)
![Website](https://img.shields.io/website?down_color=lightred&down_message=offline&label=docs&logo=readthedocs&up_color=green&up_message=online&url=http%3A%2F%2Fholonomiclibdocs.rtfd.io%2F)

# HolonomicLib

HolonomicLib is an open-source PROS (C++) library for VEX robots. The library specializes with holonomic drivetrains 
(ex. [X-Drives](https://wiki.purduesigbots.com/hardware/vex-drivetrains#x-drive) or 
[Mecanum Drives](https://wiki.purduesigbots.com/hardware/vex-drivetrains#mecanum-drive)). 

## Installation

Use the [PROS CLI](https://pros.cs.purdue.edu/v5/cli/conductor.html) to install HolonomicLib (if you installed PROS correctly, 
you do not need to install the CLI separately). 

1. Download the latest version of [HolonomicLib](https://github.com/Yessir120/HolonomicLib/releases) (it should be ``HolonomicLib@<VERSION_#>.zip``).
2. Run the following in the root of your project: 

```bash
pros conductor fetch HolonomicLib@<VERSION_#>.zip
pros conductor apply HolonomicLib
```

3. Add ``#include "HolonomicLib/API.hpp"`` to your header file

Installation steps are also found in the [documentation](https://holonomiclibdocs.readthedocs.io/en/latest/Docs/Intro/GettingStarted.html#installing-holonomiclib). 

## Usage

For step-by-step usage, follow the [**documentation**](https://holonomiclibdocs.readthedocs.io/en/latest/index.html). 

For more specific usage, follow the [**API**](https://yessir120.github.io/HolonomicLib/html/index.html). 

## Contributing
Pull requests are welcome. Experimental changes should go to the development branch until deemed stable. For major changes, please open an issue first to discuss 
what you would like to change. For both issues and PRs, please follow the issues/PR template. 

Please make sure to update tests as appropriate (that is, if tests get made).

## License
[GPL-3.0](https://choosealicense.com/licenses/gpl-3.0/)
