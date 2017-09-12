# URC-18

Hello there! Welcome to the URC-18 github repository. Here is where all the URC code for 2018 is see teamr3/URC for 2017.
The file structure of this repo is fairly simple:

| File/Directory | Purpose |
| :------------- | ------: |
| rosws | contains all the packages we have written in ros, packages that are deps should either be done with wstool or rosdep |
| scripts | contains various scripts to make things easier |
| microcontroller | contains platformio projects for each microcontroller |

## getting setup

To get setup, do the following:

1. Clone this repository

...Make sure you put it somewhere memorable, you're going to be cd-ing to it a lot.

2. From the *root* of the repo, run `scripts/setup.sh`

...The script will ask you about some things, and might prompt you to install some stuff.
...Once it's done, it'll build everything. If this fails, it probably just means we have some bad code checked in.

3. That's it!

## requirements

- a working ROS install, currently we're using kinetic.

# branches

In order to ensure working code on the master branch, keep new features/packages on their own branches. If you are re-writing something, make a new branch for the rewrite, and
base new packages off of that branch. For example:

```
+-- master
    + -- science-excel
    + -- new-auto
         + -- move_base_config
    + -- ros_control
         + new_arm
         + new_drive
```

---
*Master should always be good working code*
---

When a feature/bugfix is done on a branch, it can be merged to its parent. For example, if `new_arm` in the example was done, it would get merged to `ros_control`. When all of `ros_control` is done, it
gets merged to master. Try and use pull requests for bigger merges, especially when re-writing large portions of existing code.

# documentation

The decision has been made to use readthedocs, the url is r3-urc18.readthedocs.io.
Documentation is in the docs/ folder. Information on how to document is coming soon

