# BoVine Ingestive Behavior Measurement and Analysis (VIBMA)

Tesis de maestria: 
Red en split comportamiento ingestivo de bovinos

Instalacion:
- Instalar vscode
- Instalar docker siguiendo los pasos del siguiente tutorial 
https://code.visualstudio.com/docs/devcontainers/containers
- Instalar plugin de docker en vscode



# Socket

Si el socket de la AppAgrosavia tiene algún problema en recibir la información en Linux, en la linea 81 de ejecutable.py en lugar de <br>
UDP_IP = socket.gethostbyname(socket.gethostname()) <br>
remplazar por <br>
UDP_IP = 'ip_del_pc'  <br>
ejemplo  <br>
UDP_IP = '192.168.1.13'
