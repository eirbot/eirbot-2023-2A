# Équipe Eirbot 2023
Ce dépot recence tous les dépots (identifié comme sous module *submodule*) utilisés pour le Robot de l'équipe Eirbot participant à la coupe 2023 : THE CHERRY ON THE CAKE

# Structure mécanique
Réalisée par Lilian, le modèle 3D du robot sera disponible dès qu'il sera définitif.

# Base roulante
Pour la base roulante il y eu la confection du template Eirbot pour Mbed (Merci Copper-Bot).
Nous utilisons donc correctement mbed-cli et nous pouvons coder sur notre IDE préféré plus facilement.

Une carte de puissance V1 a été créée l'année en 2022 pour gérer BMS, rail d'alimentation et commande de moteurs BLDC (2) et DC (3).
Une V2 à vu le jour cette année dû à la nécessité d'un boost dans les rails d'alimentation.
Cette carte est dénommée **PSU-ESC**.

Nous utilisons 2 moteurs BLDC (Brush Less DC) Maxon EC45 flat 70W réduit à 1:14.

# Panier
Floris & Anabel
Utilisant une IA basée sur Yolo et dont l'apprentissage s'ait fait via Blender, le but de ce sous module est de compter le nombre de balle présentes dans le panier.
UNe Raspberry Pi est utilisée pour mener à bien ce sous module.
Contenu :
- Modèle 3D
- Code pour la Raspberry Pi
- Shield (KiCad)

# Canon
Marielle & Camille
Le but de ce sous module est de pouvoir envoyer les cerises (balles de nerf).
Il y a une conception mécanique permettant d'utiliser un seul moteur DC.
Il y a un asservissement en vitesse pour garder une dynamique stable lors du lancer.
Le contrôle de cette partie se fait via une carte Arduino Nano avec le shield.
Contenu :
- Modèle 3D
- Code pour l'Arduino Nano
- Shield (KiCad)

# Lidar
Lilian & Lucas
Nous avons prévu d'utiliser ce [Lidar](https://www.slamtec.com/en/Lidar/A1).
Pour interpréter ces informations nous prototypons avec un protocol UART en C++, mais pour l'implémentation nous avons prévu de l'implémenter sur [FPGA](https://digilent.com/shop/cmod-a7-35t-breadboardable-artix-7-fpga-module/).
Contenu :
- Code C++
- Code VHDL pour FPGA (utilisation de Vivado)
- Shield (KiCad)

# Aspirateur à balle
Guillaume

# Présentation de l'équipe :
Présents
- Léo : Président
- Lilian : Vice-Président
- Floris : Trésorier
- Guillaume : Secrétaire & Respo Part
- Louis : Respo Site
Et tout les absents sont fortement remerciés pour l'aide apportée.