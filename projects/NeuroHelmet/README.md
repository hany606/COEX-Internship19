# Neuro Helmet
This project is about to control a clever4 drone from Copter Express according to the user's emotions.
This project was a team project and my responsibility was the coding part in this repo.

[The documentation for the project in COEX clever Gitbook (Not Available now)](link)

[Video of the project](https://www.youtube.com/watch?v=uLR5NNcekfA&feature=youtu.be)

This project is controlling a drone using EEG Brain signals that is classified to a certain user's emotion through the Cortex software and use BioEcho software that emulates a real keyboard presses that is focused on a terminal that runs teleop_keyboard.py that publish the keyboard presses into a topic and the main code that runs on the drone subscribes on that topic and take a move either forward or make a rotation in Yaw direction.

[The sensor that was used](https://neurobotics.ru/catalog/nejrogarnituryi/kopiya-nejroplej-8m/)

[Cortex Software and BioEcho](https://neurobotics.ru/downloads/)

The architecture as follow:
- Cortex Software: classifies the brain signal into an emotion.
- BioEcho software: Emulate the physical keyboard.
- Teleop code: Runs on one of the terminals of the Raspberry Pi (the main Controller of the drone) from the same device that runs the 1st and the 2nd software. This listens to the keyboard presses and publishs it to key_value ros topic.
- main code: Runs on the Raspberry Pi and subscribes to key_value ros topic and make the drone move forward if the key was 'w' and rotate in yaw direction by 90 deg in anti-clockwise if the key was 's' and in order to make the movement of the drones more stable and rational to the user's emotion changes as the emotion is a continuous signal, we had made a transition signal with 'o' key value as the drone cannot do any move after performing a move unless a transition signal ('o') is sent.
