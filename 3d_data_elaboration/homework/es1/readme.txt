In questo progetto è presente il codice per compilare ed utilizzare i files del laboratorio 1 di PCL.

Installazione di PCL, CMake, g++, VTK e Eclipse-CDT (opzionale) in Ubuntu:
$ sudo add-apt-repository ppa:v-launchpad-jochen-sprickerhof-de/pcl
$ sudo apt-get update
$ sudo apt-get install libpcl-all
$ sudo apt-get install cmake g++ libvtk5.8 eclipse-cdt

Una volta installate le dipendenze, utilizzare CMake per generare  il makefile:
$ cd your_download_pat/lab1
$ cmake .
Alternativamente, per chi utilizzasse Eclipse, è possibile generare automaticamente i file di progetto.
$ cd your_download_path/lab1
$ cmake -G"Eclipse CDT4 - Unix Makefiles" -D CMAKE_BUILD_TYPE=Debug .
A questo punto potrete importarlo facilmente da Eclipse (File->Import e poi "Existing projects into workspace").
La prima volta che si importa un progetto che usa le PCL è necessario aspettare qualche minuto affinché Eclipse termini l'indexing (tenete d'occhio la percentuale in basso a destra).


