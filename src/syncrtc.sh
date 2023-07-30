#!/bin/bash
# MQTT broker details
USERNAME="ameba"
PASSWORD="ameba"
TOPIC="timestamp"
# Get the IP address of the current Linux system
IP_ADDRESS=$(hostname -I | awk ’{print $1}’)
# Get the current date and time in the desired format
DATE_TIME=$(date +%s)
# Publish the date and time with IP address via MQTT
mosquitto_pub -h "$IP_ADDRESS" -t "$TOPIC" -u "$USERNAME" -P "$PASSWORD" -m "$DATE_TIME"