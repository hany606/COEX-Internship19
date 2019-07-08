#!/bin/bash
nmcli c up CLEVER-6145_Hany	# Add the name of the drone access point here instead
sshpass -p 'raspberry' ssh pi@192.168.11.1	# Add the password of raspberry pi here instead it is more secured not to make this method as it saves the password as plain text but now it is not that important
