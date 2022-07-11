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

## Usage

```python
import foobar

# returns 'words'
foobar.pluralize('word')

# returns 'geese'
foobar.pluralize('goose')

# returns 'phenomenon'
foobar.singularize('phenomena')
```

## Contributing
Pull requests are welcome. For major changes, please open an issue first to discuss what you would like to change.

Please make sure to update tests as appropriate.

## License
[MIT](https://choosealicense.com/licenses/mit/)
