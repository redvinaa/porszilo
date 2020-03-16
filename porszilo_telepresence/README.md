__Sziasztok,__  

FONTOS: mindenek előtt a master-t kell elindítani: `roscore`  

Csináltam pár bag filet, ezek infót küldenek a mi node-unknak, 
ezeket lejátszva tudjátok tesztelni a programot, 
mintha valóban mozogna a robot.  
Amit küldünk neki majd pozíciót nem fogja látni, mivel ez
csak egy felvétel, minden másra viszont jó.

Ezekkel lehet elindítani a bag fileokat (nyilván egyszerre csak egyet).  
Az `-l` option loop-ot jelent, szóval végtelenítve van.  

`rosbag play -l $(rospack find porszilo_telepresence)/bags/porszilo_standing.bag`  
`rosbag play -l $(rospack find porszilo_telepresence)/bags/porszilo_moving.bag`  

Ezzel lehet futtatni a programunkat.  
`rosrun porszilo_telepresence basic_node.py`  

Ezzel lehet vizualizálni a beérkező infót:  
`rosrun rviz rviz -d $(rospack find porszilo_telepresence)/config/porszilo.rviz`  

Ezzel lehet kirajzolni a hálózatot.  
`rosrun rqt_graph rqt_graph`  

Ne felejtsétek a "~/.bashrc"-be beleírni a következő parancsokat, különben a hivatkozások nem fognak működni:  
`source /opt/ros/${ROS_DISTRO}/setup.bash`  
`source <catkin_ws>/devel/setup.bash`  

A real-time szimulációt a következő paranccsal tudjátok elindítani:  
`roslaunch porszilo porszilo.launch`

Lásd: `$(rospack find porszilo_telepresence)/example_screen.png`  
Lásd: `$(rospack find porszilo_telepresence)/example_screen_2.png`


Megbeszélt feladatok:
	- webes megjelenítés
	- 
