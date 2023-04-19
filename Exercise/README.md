# simple_dmp

This is an (almost) minimal implementation of DMPs in Cartesian space (both position and quaternion) and in Joint space, with an optional (default disabled) roto-dilatation term.


## Dependencies
This project depends on:
* NumPy
* NumPy-Quaternion
* Matplotlib
* Pandas

To install these, run <code>pip3 install --user numpy numpy-quaternion matplotlib pandas</code>.	

## Running
To the run the project, navigate to the root directory and call <code>python3 main.py</code>. This will first train and plot a DMP in joint space and then do the same in Cartesian space for the provided demonstration file, demonstration.csv.

