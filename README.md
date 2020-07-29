# agrosaviaRepositorio

Proyecto de Grado:
Monitorio de comportamientoIngestivo Bovinos en Pastoreo

pip install -r requirements.txt

tutorial-env\Scripts\activate.bat
On Unix or MacOS, run:

source tutorial-env/bin/activate



# Socket

Si el socket de la AppAgrosavia tiene algún problema en recibir la información en Linux, en la linea 81 de ejecutable.py en lugar de <br>
UDP_IP = socket.gethostbyname(socket.gethostname()) <br>
remplazar por <br>
UDP_IP = 'ip_del_pc'  <br>
ejemplo  <br>
UDP_IP = '192.168.1.13'
