#!/bin/bash
# Pull the official AutoDRIVE RoboRacer images for IROS 2026.
# Usage: ./scripts/pull_images.sh [TAG]   (default: 2026-iros-practice)
set -e
TAG="${1:-2026-iros-practice}"
docker pull "autodriveecosystem/autodrive_roboracer_sim:${TAG}"
docker pull "autodriveecosystem/autodrive_roboracer_api:${TAG}"
