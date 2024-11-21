# **TurtleBot3 Student Master 2 Mecatronic with ROS2 in Ubuntu 22.04**


The team is composed of 6 students: 

* Azad ROJOA
* Aimy MADELPECH
* Antoine BEVILACQUA
* Valentin LEGLISE
* Maxence TURPAULT
* Maxime PAOLANTONI


## Chalenge du TurtleBot

L'objectif de ce projet est mettre en place différentes phases de déplacement du robot. 

* La phase 1 : 

L'humain commence l'épreuve en étant en face du robot à une distance
de 50 cm. Le robot doit suivre l'humain en maintenant une distance comprise entre 20cm minimum et 1m
maximum. Pour ête validé il doit avoir une démarche pas égale et doit avoir une distance minimum de marche demandée 4
mètres.

Consigne: doit passer à travers une porte, et une fois arriver le robot doit s'arrêter
pendant une durée d'au moins 3 secondes. Le robot doit alors comprendre que la phase 1 est terminée et
passer à la phase2.

* Phase 2: 

Le robot doit repartir et naviguer en totale autonomie jusqu'à son point de départ. Sur le retour, vous
rajouterez jusqu'à :

    - 1 obstacle statique sur son chemin de retour
    - 1 obstacle dynamique (typiquement un humain qui lui coupe la route)
    - 1 obstacle qui bloque complètement le passage prévu par le robot (il faut qu'il ait la possiblité d'arriver à destination par un autre chemin)

Si le robot arrive à destination (à +-20cm, +-15°) la phase 2 est validée.


* Phase 3 : Dock


Le robot doit chercher où se trouve sa base et s'y accoster. La position grossière de la base est connue mais
cette partie n'est validée que si le robot réussi un accostage précis sans contact : la distance entre le robot et
la base soit être supériere à 5mm et inférieure à 2cm. Vous avez toute liberté pour choisir un objet qui
représentera la base du robot. Un pot de peinture par exemple serait un choix pertinent (la symétrie radiale
peut simplifier la détection).


# Rendu

* Dépot Git à rendre avec le code inclus
* 1 Vidéo démontrant le challenge
* Inclure un fichier Result.txt
