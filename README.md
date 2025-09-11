# C implementation of Klampt math modules

## Usage

For documentation, see: https://motion.cs.illinois.edu/software/klampt/latest/pyklampt_docs/klampt.math/

Only `vectorops, so3, se3` packages are implemented.

There's some test code for a UR driver that's probably never going to be finished...

## Setup

Clone this repository: `git clone https://github.com/mass2010chromium/motion-c.git; cd motion-c`

To build and install python package (using C implementation): `make`

To install C headers: `sudo make install`

## Debugging

If you don't like to see segfaults in your python code, edit `setup.py` and uncomment the first definitions of `all_macros` (and comment the second definition out) to enable error checking.
