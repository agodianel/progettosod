Questo cartella contiene il codice implementato su Ameba AMB23 che raccoglie dati da diversi sensori (SGP30, MAX30102, RTC) e li invia tramite MQTT a un broker remoto. Il progetto utilizza il sistema operativo FreeRTOS per gestire i compiti in modo asincrono e garantire un funzionamento efficiente.

# Funzionalità principali

## Inizializzazione di WiFi e Sensori
Il programma si avvia inizializzando la connessione WiFi. Se la connessione non riesce, il dispositivo proverà a connettersi nuovamente. Successivamente, vengono inizializzati i sensori SGP30 e MAX30102.

## Sincronizzazione dell'Orario
Prima di iniziare la raccolta dati, il dispositivo attende di ricevere il timestamp iniziale dal topic MQTT "timestamp". Questo timestamp viene utilizzato per sincronizzare l'orologio in tempo reale (RTC) del dispositivo.

## Task del Sensore
Il task del sensore raccoglie periodicamente i dati dai sensori SGP30 e MAX30102 e li invia ad una coda condivisa chiamata "sensorQueue". I dati raccolti includono i valori TVOC, eCO2, frequenza cardiaca, livello di ossigeno nel sangue (SpO2) e il timestamp corrente.

## Task MQTT
Il task MQTT gestisce la connessione e la comunicazione con il broker MQTT. Se la connessione MQTT viene persa, il task tenta di riconnettersi. Quando è connesso, il task riceve i dati dai sensori dalla coda "sensorQueue" e li invia a due topic MQTT: "sensor" per i dati attuali e "history" per i dati storici.

## Gestione dati storici
Se la connessione MQTT viene persa, i dati dai sensori vengono temporaneamente salvati nella coda "historyQueue". Quando la connessione viene ripristinata, i dati storici vengono inviati al topic "history".

## Ulteriori informazioni
Per ulteriori dettagli sul funzionamento del codice e sull'utilizzo dei singoli sensori, fare riferimento ai commenti nel codice sorgente. Per configurare correttamente il broker MQTT e l'ambiente di rete, consultare la documentazione delle rispettive librerie utilizzate.