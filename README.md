# Extraction d'informations de scans LiDAR intérieurs

### Projet de fin d'études de Master 2 Géométrie et Informatique Graphique

### Réalisé par Colas Claudet, Yoann Foulon et Rachel Glaise

- [Résumé](#r%c3%a9sum%c3%a9)
- [Dépendances](#d%c3%a9pendances)
- [Installation](#installation)
  - [Installer les outils de compilation](#installer-les-outils-de-compilation)
  - [Installer Qt et QtCreator](#installer-qt-et-qtcreator)
  - [Installer OpenCV et OpenCV Contrib:](#installer-opencv-et-opencv-contrib)
  - [Installer PCL (Point Cloud Library)](#installer-pcl-point-cloud-library)
  - [Configuration du projet](#configuration-du-projet)

## Résumé

Les nuages de points sont très utilisés dans de nombreux domaines pour leurs multiples applications. Dans ce contexte, nous avons manipulé des nuages de points provenant de scans intérieurs de pièces pour créer des modélisations simplifiées de ces pièces. Pour cela, nous avons utilisé Point Cloud Library, qui dispose de nombreuses fonctions de manipulation de nuages de points, mais aussi l’algorithme KIPPI, que nous avons implémenté à l’aide de la librairie OpenCV, et qui permet d’extraire des informations sur des images 2D. Ce projet est réalisé dans le cadre de notre Projet de Fin d’Étude et fait suite au TER du même nom réalisé l’année précédente.

## Dépendances

Ce projet nécéssite les éléments suivants pour fonctionner:

- PCL (Point Cloud Library)
- OpenCV et ses contributions
- CMake
- Make
- g++
- Qt
- les dépendances sur lesquelles reposent les éléments ci-dessus

Notre programme est compatible uniquement avec un système GNU/Linux. L'installation des éléments ci-dessus dépendant des distributions, nous couvrirons dans la partie suivante uniquement un exemple d'installation sur la distribution Ubuntu 18.04, système que nous avons utilisé pour le développement.

## Installation

### Installer les outils de compilation

```
sudo apt install build-essential
```

### Installer Qt et QtCreator

```
sudo apt-get install qtcreator qt5-default \
qt5-doc qt5-doc-html qtbase5-doc \
qtbase5-doc-html qtbase5-examples
```

### Installer OpenCV et OpenCV Contrib:

1. Dépendances:

```
sudo apt-get install cmake git libgtk2.0-dev \ 
pkg-config libavcodec-dev libavformat-dev \
libswscale-dev libtbb2 libtbb-dev \
libjpeg-dev libpng-dev libtiff-dev \
libdc1394-22-dev
```

2. Téléchargement:

```
cd ~/<my_working_directory>
git clone https://github.com/opencv/opencv.git
git clone https://github.com/opencv/opencv_contrib.git
```

3. Création du dossier build:

```
cd ~/opencv
mkdir build
cd build
```

4. Configuration:

```
cmake -D CMAKE_BUILD_TYPE=Release \
-D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules \
-D CMAKE_INSTALL_PREFIX=/usr/local ..
```

5. Build: 

```
make -jX #Remplacer X par votre nombre de coeurs du processeur
```

6. Installation:

```
sudo make install
```

### Installer PCL (Point Cloud Library)

```
sudo apt install libpcl-dev
```

### Configuration du projet

1. Télécharger le projet:

```
git clone https://github.com/colasclaudet/PFE_PCL.git
```

2. Créer un dossier build:

```
cd PFE_PCL
mkdir build
```

3. Configuration sous QtCreator:

Ouvrir ```ScanLIDAR.pro``` avec QtCreator et créez la configuration suivante dans Projects > Build:

![Configuration du projet sous QtCreator](/images/config.png)

4. Execution

Lancez le projet avec QtCreator. Vous devriez obtenir l'interface suivante:

![Interface générale](/images/interface.png)

L'éxécution du programme se déroule en plusieurs phases successives:

- Import d'un nuage de point avec le bouton "Import"
- EXPLICATION SLIDERS
- (Optionnel) Affichage du nuage de point sans traitement avec le bouton "Affichage basique"
- Segmentation du nuage en plan avec le bouton "Segmenter":
  ![Segmentation](/images/segmentation.png)
- Tri des plans (Attribution aux murs, plafond, sol) avec le bouton "Trier":
  ![Tri des plans](/images/tri.png)
- Modélisation de la pièce avec le bouton "Modéliser":
  ![Modelisation](/images/modelisation.png)

Chaque nouvelle étape devient accessible une fois la fenêtre de l'étape précédente fermée.
