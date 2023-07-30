# Cartella Rpi4

Questa cartella contiene una serie di script per la gestione di un Raspberry Pi 4 (Rpi4) utilizzando diverse funzionalità come MQTT e interazione con un database.

## Script Disponibili

1. `start_mqtt.sh`: Avvia il servizio MQTT utilizzando il comando `sudo systemctl start mosquitto.service`.
2. `stop_mqtt.sh`: Ferma il servizio MQTT utilizzando il comando `sudo systemctl stop mosquitto.service`.
3. `syncrtc.sh`: Legge l'ora di sistema in formato Unix e la invia al topic "timestamp" tramite MQTT per sincronizzare il tempo su altri dispositivi.
4. `write_db.sh`: Scrive dati su InfluxDB utilizzando richieste HTTP POST.
5. `stop_db.sh`: Verifica se lo script `write_db.sh` è in esecuzione e, se presente, lo interrompe.

## Requisiti
1. Assicurarsi di avere i permessi di esecuzione per gli script desiderati, in caso contrario, utilizzare il comando seguente per concedere i permessi:

   ```bash
   chmod +x nome_script.sh

`sudo chmod +x nome_file.sh`

Gli script start_mqtt.sh e stop_mqtt.sh contengono il comando ’sudo’, ciò vuol dire
che viene richiesta la password che viene impostata quando viene flashato la SD,
ma su Raspberry Pi OS per semplicità esiste una direttiva per fare in modo
che sudo non chieda la password dell’utente che sta eseguendo il comando.
La direttiva si trova nel file `/etc/sudoers.d/010_pi-nopasswd`


