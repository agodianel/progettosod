#!/bin/bash
# Check if the write_db.sh script is running
if pgrep -x "write_db.sh" > /dev/null; then
# Stop the write_db.sh script
pkill -f "write_db.sh"
echo "write_db.sh script has been stopped."
else
echo "write_db.sh script is not running."
fi