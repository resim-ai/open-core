#!/bin/sh
#
# Copyright 2025 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

PORT=12345
HOST=127.0.0.1

if [ "$MCB_ROLE" = "SENDER" ]; then 
    echo "Sending greeting.txt to $HOST:$PORT..."
    # Keep trying every second until successful
    while ! nc -w1 "$HOST" "$PORT" < greeting.txt 2>/dev/null; do
        sleep 1
    done
    echo "Message sent successfully!"
    
elif [ "$MCB_ROLE" = "RECEIVER" ]; then
    echo "Listening on port $PORT (will restart after each connection)..."
    while true; do
        nc -l -p "$PORT" && break  # Exit after first successful connection
    done
else
    echo "Error: MCB_ROLE must be either SENDER or RECEIVER"
    exit 1
fi
