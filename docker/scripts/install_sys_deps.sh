#!/bin/bash
set -o pipefail

# Update system packages
apt-get update && apt-get upgrade -y

# Install necessary system packages
apt-get install -y \
    git \
    git-lfs \
    curl \
    python3-pip \
    python3-vcstool \
    python-is-python3 \
    qtbase5-dev \
    bash-completion

# Configure git to use HTTPS
git config --global url.https://github.com/.insteadOf git@github.com:
git config --global advice.detachedHead false

# Clean up
rm -rf /var/lib/apt/lists/*
