# RYCSV - 2020II
## Proyecto - Rover de exploración

<br />
<br />

**Autores:**
- Jesús Caballero
- Sebastián Cortés
- Jhohan Higuera
- ALejandro Montés
 
**Profesor:** Flavio Prieto

**Tutor:** Jose Fajardo

<br />
<br />

**Comando:**

_Lanzar modelo en Gazebo:_

    roslaunch rover_RYCSV empty_map.launch
    
    
_Nodo de control:_

    rosrun rover_RYCSV control_path_coord_node.py 
    
    
_Para cambiar a control lineal en el archivo:

    TF_node.py 
    
    Descomentar
    
    from class_TF_lineal import TF     ##lineal
    
    Comentar:
    
    from class_TF import TF              ##Polar

_Se deben instalar los paquetes:

http://wiki.ros.org/robot_pose_ekf

http://wiki.ros.org/gps_common
    
