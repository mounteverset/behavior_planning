#!/usr/bin bash
echo -e "Terminal_ROSbot:\c"
tty
echo "connecting to ROSbot..."
#sshpass must be installed

sshpass -p husarion ssh husarion@192.168.188.198 "bash bt/microros_restart.sh"

if [ $? -eq 0 ]; then
    echo -e "\033[0;32mRestarted!!"
    exit 0
else
    echo -e "\033[0;31mNO CONNECTION"
    exit 1
fi

