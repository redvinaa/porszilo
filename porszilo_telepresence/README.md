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


Szóval ezeket kéne még elintézni: 
	- a kamera képe legyen színes, ahova lehet kattintani az pedig legyen valami adott szín (pl. piros, nyilván átlátszó kicsit), ezt utána el lehet mosni (ha anélkül csúnya)
	- most úgy van beállítva a `goal` orientációja, hogy a végén mindig egyenesen néz a robot, ezzel azt lehetne csinálni, hogy megnézzük hogy a mostani pozícióhoz képest hol van a cél és légvonalban összekötjük a két pontot. Amerre néz ez a vektor, az legyen a végső orientáció (ha valakinek van jobb ötlete, mondja)
	- mivel a fenti megoldás nem nagyon felhasználóbarát, kéne valami olyasmit csinálni, hogy a robot tudjon helyben forogni (ahogy abban a videóban volt amit a Tibor küldött), ezt fogalmam sincs hogy lehetne megoldani, valszleg a billentyűzetet nem tudjuk használni mert úgy tabletről nem fog működni
