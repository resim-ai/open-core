#!/bin/sh
set -euxo pipefail
echo "Welcome to ReSim Log Ingestion"

# Exit codes:
NO_OUTPUT_DIRECTORY=10
NO_INPUT_DIRECTORY=11
COPY_FAILED=20
NO_FILES_EXISTED=30

# Define input and output directories
INPUT_DIR="/tmp/resim/inputs"
OUTPUT_DIR="/tmp/resim/outputs"
ERROR_FILE="$OUTPUT_DIR/error.txt"

# Check if input or output directory exists; otherwise exit:
if [ ! -d "$OUTPUT_DIR" ]; then
  echo "Outputs directory does not exist"
  exit $NO_OUTPUT_DIRECTORY
fi

if [ ! -d "$INPUT_DIR" ]; then
  echo "Inputs directory does not exist" | tee "$ERROR_FILE" # we have an outputs directory, we can write to error.txt
  exit $NO_INPUT_DIRECTORY
fi

# List contents of input directory
echo "Contents of /tmp/resim/inputs directory:"
ls -lh "$INPUT_DIR"

# Attempt to copy contents from input to output directory
if cp -r "$INPUT_DIR"/* "$OUTPUT_DIR"/; then
  # Count the number of files copied
  FILE_COUNT=$(ls -1 "$INPUT_DIR" | wc -l)
  if [ "$FILE_COUNT" -eq 0 ]; then
    # If no files were copied, write error to error file
    echo "No files existed in inputs directory" | tee "$ERROR_FILE"
    exit $NO_FILES_EXISTED
  fi
  echo "Ingested $FILE_COUNT files"
else
  # If copy fails, write error to error file
  echo "Failed to copy files from inputs to outputs" | tee "$ERROR_FILE"
  exit $COPY_FAILED
fi
