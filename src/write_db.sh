#!/bin/bash
# Get the IP address of the current Linux system
IP_ADDRESS=$(hostname -I | awk ’{print $1}’)
# MQTT Broker settings
MQTT_BROKER=$(hostname -I | awk ’{print $1}’)
MQTT_TOPIC="history"
# InfluxDB settings
INFLUXDB_SERVER=$(hostname -I | awk ’{print $1}’)
INFLUXDB_DB="ameba"
INFLUXDB_URL="http://$INFLUXDB_SERVER:8086/write?db=
$INFLUXDB_DB"
INFLUXDB_USER=""
INFLUXDB_PASSWORD=""

# Function to process and write the MQTT message to InfluxDB

write_to_influxdb() {
local message=$1
local tvoc
local eco2
local hr
local spo2
local timestamp
IFS=’,’ read -ra values <<< "$message"
if [[ "${#values[@]}" -eq 5 ]]; then
tvoc=${values[0]}
eco2=${values[1]}
hr=${values[2]}
spo2=${values[3]}
timestamp=${values[4]}
# Format the data as InfluxDB line protocol with
# measurement name "data"
data="data
eco2=$eco2,hr=$hr,spo2=$spo2,timestamp=$timestamp,tvoc=$tvoc"
# Perform HTTP POST to InfluxDB
curl -i -XPOST "$INFLUXDB_URL" --data-binary "$data" --user "$INFLUXDB_USER:$INFLUXDB_PASSWORD"
else
echo "Invalid message format: $message"
fi
}
# Subscribe to MQTT topic and process messages
INFLUXDB_URL="http://$INFLUXDB_SERVER:8086/write?db=
$INFLUXDB_DB"
INFLUXDB_USER=""
INFLUXDB_PASSWORD=""
# Function to process and write the MQTT message to InfluxDB
write_to_influxdb() {
local message=$1
local tvoc
local eco2
local hr
local spo2
local timestamp
IFS=’,’ read -ra values <<< "$message"
if [[ "${#values[@]}" -eq 5 ]]; then
tvoc=${values[0]}
eco2=${values[1]}
hr=${values[2]}
spo2=${values[3]}
timestamp=${values[4]}

# Format the data as InfluxDB line protocol with
# measurement name "data"
data="data eco2=$eco2,hr=$hr,spo2=$spo2,timestamp=$timestamp,tvoc=$tvoc"

# Perform HTTP POST to InfluxDB
curl -i -XPOST "$INFLUXDB_URL" --data-binary "$data"
--user "$INFLUXDB_USER:$INFLUXDB_PASSWORD"
else
echo "Invalid message format: $message"
fi
}
# Subscribe to MQTT topic and process messages