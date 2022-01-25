# doublemag-fixture
stuff for my double magnet fixture

## Working with z-offset
When  the sample is not at the same height as the magnets, i.e. there is an offset in the z axis, a different control mode is used.
Z offset at some key landmarks are listed:

- To bottom side of t slot aluminum: 33.66 mm
- To bottom side of t slot aluminum plus additional 1/16": 35.2 mm
- To bottom side of t slot aluminum plus additional 1/8": 36.84 mm

## Issues when running on ubuntu:

- https://askubuntu.com/questions/308128/failed-to-load-platform-plugin-xcb-while-launching-qt5-app-on-linux-without >> run `sudo apt-get install --reinstall libxcb-xinerama0`