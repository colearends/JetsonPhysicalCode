This Repository is made to hold all the jetson/arduino physical code and HMI app resources for the autonomous golf cart project. Refer to Branches to find files. 

Steps for running the hmi:

1) Download the zip file and arduino file
2) upload sketch to arduino
3) ensure the serial monitor is off
4) turn on bluetooth on computer and search hc-05
5) enter password :1234
6) go to settings -> bluetooth and devices -> bluetooth -> devices -> more devices and printer settings -> right click hc-05 -> properties -> hardware -> com 10 -> ok
7) open python script named relay.py
8) open a new terminal and enter "python relay.py"
9) terminal will say running on server 8217
10) launch app.js using a second terminal
11) in the second terminal run "npx http-server -p 8080" **app server cannot be the same as the python server**
12) after entering that code, a link should appear in the terminal.
13) control select http://127.0.0.1:8080
14) the app should launch in browser
15) press center button on d-pad to arm device.
16) all buttons should work properly.

17) to stop server, control C just the app terminal, the python terminal can stay running indefinately.

18) to open a new server for another try, use a different server number , example 8081
