#!/usr/bin/env bash
set -e

function nexist {
   if hash $1 2>/dev/null; then
      echo 0
   else
      echo 1
   fi
}

if [ "$EUID" -ne 0 ]; then
   echo "[!] I need root to work correctly, please run me as 'sudo scripts/setup.bash' instead"
   exit 1
fi

if [ $(nexist roscore) -eq 1 ]; then
   echo "[!] I couldn't find roscore, which probably means you either a) didn't install ros, or b) didn't source /opt/ros/kinetic/setup.bash"
   echo "[!] Please ensure ros is installed before running this script"
   exit 1
fi

if [ ! -d rosws ]; then
   echo "[!] Please run me at the root of the repo!"
   exit 1
fi

if [ $(nexist dialog) -eq 1 ]; then
   echo "[*] Please wait while InstallShield (tm) prepares the URC18 setup wizard..."
   apt install -y dialog > /dev/null
   sleep 0.25
fi

dialog --backtitle "URC18 setup wizard" --title "Welcome!" --msgbox "Welcome to the InstallShield setup wizard for URC18. This program will guide you through installing URC18 and getting it to work" 9 60

NEED_WSTOOL=0
NEED_PIO=0

echo "0" | dialog --backtitle "URC18 setup wizard" --title "Checking dependencies" --gauge "Checking wstool..." 10 70 0
NEED_WSTOOL=$(nexist wstool)
#echo $(nexist wstool)
sleep 0.5
echo "50" | dialog --backtitle "URC18 setup wizard" --title "Checking dependencies" --gauge "Checking pio..." 10 70 0
NEED_PIO=$(nexist pio)
sleep 0.5
echo "100" | dialog --backtitle "URC18 setup wizard" --title "Checking dependencies" --gauge "Checking pio..." 10 70 0

RESULT=""
ANY=1

if [ $NEED_WSTOOL -eq 1 ]; then
    RESULT="wstool"
    if [ $NEED_PIO -eq 1 ]; then
       RESULT="wstool and platformio"
    fi
else
    if [ $NEED_PIO -eq 1 ]; then
       RESULT="platformio"
    else
       ANY=0
    fi
fi

if [ $ANY -eq 1 ]; then
   dialog --backtitle "URC18 setup wizard" --title "Requirements" --msgbox "You need the following dependencies, if you continue I will install them for you:\n $RESULT \n Press CTRL+c to cancel, or press OK to continue" 8 60
   dialog --clear
   if [ $NEED_WSTOOL -eq 1 ]; then
      echo "[*] Installing wstool with 'apt install -y python-wstool'"
      apt install -y python-wstool
   fi
   if [ $NEED_PIO -eq 1 ]; then
      echo "[*] Installing platformio with 'pip install -U platformio'"
      pip install -U platformio
   fi
fi

dialog --backtitle "URC18 setup wizard" --title "Confirm" --msgbox "I am now ready to setup the workspace, press OK to continue!" 8 60

function progress {
	echo $2 | dialog --backtitle "URC18 setup wizard" --title "Working" --gauge "$1" 10 60 0
}

progress "Creating workspace" 0
if [ ! -d rosws/src ]; then
   runuser $SUDO_USER -c 'mkdir rosws/src'
fi
if [ ! -f rosws/src/CMakeLists.txt ]; then
   runuser $SUDO_USER -c 'catkin_init_workspace rosws/src' > /dev/null
fi
sleep 0.25
progress "Pulling wstool dependencies" 25
if [ ! -f rosws/src/.rosinstall ]; then
   runuser $SUDO_USER -c 'wstool init rosws/src'
fi
runuser $SUDO_USER -c 'wstool update -t rosws/src' > /dev/null
progress "Installing rosdeps" 50
rosdep install --from-paths rosws/src -q -y --rosdistro kinetic --ignore-src > /dev/null
sleep 0.25
progress "Running catkin_make" 75
pushd . > /dev/null
cd rosws
runuser $SUDO_USER -c 'catkin_make' > /dev/null
popd > /dev/null
progress "Done" 100
sleep 0.25

dialog --backtitle "URC18 setup wizard" --title "Done!" --msgbox "URC18 has been succesfully installed! \n\n[ ] start URC18 after finishing\n[ ] view README\n\n (just kidding, you can't view the README!)" 10 50
clear

echo "[*] Done!"
