#!/bin/bash
# Deployment script for GitHub Pages - requires GIT_USER environment variable

set -e

if [ -z "$GIT_USER" ]; then
  echo "Error: GIT_USER environment variable not set"
  echo "Usage: GIT_USER=your-github-username bash deploy.sh"
  exit 1
fi

echo "Building Docusaurus site..."
cd website
pnpm install
pnpm build

echo "Deploying to GitHub Pages..."
cd ..

GIT_USER=$GIT_USER \
  USE_SSH=false \
  yarn deploy

echo "Deployment complete! Site will be live at https://$GIT_USER.github.io/panaversity-hackathon/"