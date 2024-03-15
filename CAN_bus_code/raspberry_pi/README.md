# Mode d’emploi pour module CAN basé sur MCP2515 (Raspberry avec terminal : Zero, 3, 4)
https://domoticx.com/raspberry-pi-can-bus-communicatie-gpio/ 
 
Pour charger les drivers dans l’interface SPI :
`$ sudo nano /boot/config.txt`
et ajouter le code suivant à la fin du fichier:
```
dtparam=spi=on
dtoverlay=mcp2515-can0,oscillator=8000000,interrupt=25,spimaxfrequency=500000
dtoverlay=spi-bcm2835-overlay
```

La fréquence est écrite sur l’oscillateur du module (8.000 = 8MHz).

## Installer can-utils
`$ sudo apt-get install can-utils`
Ensuite, *reboot la Raspberry* !
Si bien installé:
`$ ls /sys/bus/spi/devices/spi0.0/net` affiche
```
can0
```

## Démarrer l’interface CAN
`$ sudo /sbin/ip link set can0 up type can bitrate 500000`

Vérifier avec `$ifconfig`
## Lancement automatique de l’interface au démarrage de la RP
`$ sudo nano /etc/network/interfaces`, ajouter à la fin:
```
auto can0
iface can0 inet manual
    pre-up /sbin/ip link set can0 type can bitrate 500000 triple-sampling on restart-ms 100
    up /sbin/ifconfig can0 up
    down /sbin/ifconfig can0 down
```

# Utilisation
-	`candump any` – écouter n’importe quel appareil canbus connecté
-	`candump can0` – seulement écouter l’appareil can0
-	`cansend` – Send a single frame.
-	`cansend can0 020#01` – Send 1 byte (1st) to device ID 0x20 (32)
-	`candump` – Dump can packets – display, filter and log to disk.
-	`canplayer` – Replay CAN log files.
-	`cangen` – Generate random traffic.
-	`canbusload` – display the current CAN bus utilisation.
-	`candump can0,0x21:7ff` = only listen on canbus device can0 and canbus device id 0x20 (32)
