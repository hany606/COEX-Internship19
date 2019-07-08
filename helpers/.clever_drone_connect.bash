#!/bin/bash
connect_full(){
  nmcli c up CLEVER-3808-6145_Hany  # Add the name of the drone access point here instead
  sshpass -p 'raspberry' ssh pi@192.168.11.1  # Add the password of raspberry pi here instead it is more secured not to make this method as it saves the password as plain text but now it is not that important
}

connect_network(){
  nmcli c up CLEVER-3808-6145_Hany  # Add the name of the drone access point here instead
}

connect_ssh(){
  sshpass -p 'raspberry' ssh pi@192.168.11.1  # Add the password of raspberry pi here instead it is more secured not to make this method as it saves the password as plain text but now it is not that important

}

disconnect_network(){
  nmcli c down CLEVER-3808-6145_Hany
}


if [ "$1" == "-f" ] | [ $# -eq 0 ]
then
	connect_full
fi

if [ "$1" == "-n" ]
then
  connect_network
fi

if [ "$1" == "-s" ]
then
  connect_ssh
fi

if [ "$1" == "-d" ]
then
  disconnect_network
fi


