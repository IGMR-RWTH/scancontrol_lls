#!/bin/bash
set -euo pipefail

# Get the directory of the script
SCRIPT_DIR="$(dirname "$(realpath "${BASH_SOURCE[0]}")")"
echo "Script directory: $SCRIPT_DIR"

# Define the URL and the target directory relative to the script's directory
URL="https://software.micro-epsilon.com/scanCONTROL-Linux-SDK-1-0-1.zip"
TARGET_DIR="$SCRIPT_DIR/ext/me_sdk"
ZIP_FILE="$SCRIPT_DIR/me_sdk.zip"

# Create the target directory if it doesn't exist
mkdir -p "$TARGET_DIR"

# Download the file (-L follows redirects, --fail errors out on HTTP errors)
echo "Downloading SDK from $URL..."
curl -L --fail "$URL" --output "$ZIP_FILE"

# Unzip the downloaded file to the target directory (-o overwrites on re-run)
echo "Extracting SDK files..."
unzip -o "$ZIP_FILE" -d "$TARGET_DIR"

# remove the zip file after extraction
rm "$ZIP_FILE"
echo "Download and extraction complete."

# Copy the CMake build files to the SDK directories
echo "Copying CMake build files..."
cp "$SCRIPT_DIR/patches/libllt/CMakeLists.txt" "$TARGET_DIR/libllt/"
cp "$SCRIPT_DIR/patches/libllt/package.xml" "$TARGET_DIR/libllt/"
cp "$SCRIPT_DIR/patches/libmescan/CMakeLists.txt" "$TARGET_DIR/libmescan/"
cp "$SCRIPT_DIR/patches/libmescan/package.xml" "$TARGET_DIR/libmescan/"

# Apply the llt.h patch. The diff paths are relative to the repo root, so apply
# with -p0 from SCRIPT_DIR. Guard with a dry-run so re-running is idempotent.
LLT_PATCH="$SCRIPT_DIR/patches/libllt/llt.h.patch"
echo "Applying llt.h patch..."
if patch -p0 -d "$SCRIPT_DIR" --forward --dry-run <"$LLT_PATCH" >/dev/null 2>&1; then
  patch -p0 -d "$SCRIPT_DIR" --forward <"$LLT_PATCH"
  echo "llt.h patch applied."
else
  echo "llt.h patch already applied (or not applicable); skipping."
fi
echo "Finished SDK setup."
